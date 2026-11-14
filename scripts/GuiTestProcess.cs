// Disposable GUI-process launcher. Never switches desktops or sends real input.
// The job owns ONLY the test process tree; closing it also handles timeout cleanup.
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;

namespace Loom
{
    public sealed class GuiTestProcess : IDisposable
    {
        private IntPtr desktop, process, job;
        public uint ProcessId { get; private set; }
        public string DesktopName { get; private set; }

        [StructLayout(LayoutKind.Sequential)]
        private struct SecurityAttributes
        {
            public int length;
            public IntPtr descriptor;
            [MarshalAs(UnmanagedType.Bool)] public bool inherit;
        }
        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
        private struct StartupInfo
        {
            public int cb;
            public string reserved, desktop, title;
            public uint x, y, width, height, columns, rows, fill, flags;
            public ushort show, reservedBytes;
            public IntPtr reservedData, stdin, stdout, stderr;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct ProcessInfo
        {
            public IntPtr process, thread;
            public uint processId, threadId;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct BasicLimits
        {
            public long processTime, jobTime;
            public uint flags;
            public UIntPtr minWorkingSet, maxWorkingSet;
            public uint activeProcesses;
            public UIntPtr affinity;
            public uint priority, scheduling;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct IoCounters
        {
            public ulong readOps, writeOps, otherOps, readBytes, writeBytes, otherBytes;
        }
        [StructLayout(LayoutKind.Sequential)]
        private struct ExtendedLimits
        {
            public BasicLimits basic;
            public IoCounters io;
            public UIntPtr processMemory, jobMemory, peakProcessMemory, peakJobMemory;
        }

        [DllImport("user32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr CreateDesktopW(string name, IntPtr device, IntPtr mode,
            uint flags, uint access, IntPtr attributes);
        [DllImport("user32.dll", SetLastError = true)]
        private static extern bool CloseDesktop(IntPtr handle);
        [DllImport("user32.dll", SetLastError = true)]
        private static extern IntPtr GetProcessWindowStation();
        [DllImport("user32.dll", SetLastError = true)]
        private static extern IntPtr OpenInputDesktop(uint flags, bool inherit, uint access);
        [DllImport("user32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern bool GetUserObjectInformationW(IntPtr handle, int index,
            StringBuilder buffer, uint bytes, out uint needed);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern bool CreateProcessW(string application, StringBuilder command,
            IntPtr processAttributes, IntPtr threadAttributes, bool inherit, uint flags,
            IntPtr environment, string cwd, ref StartupInfo startup, out ProcessInfo info);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr CreateFileW(string name, uint access, uint sharing,
            ref SecurityAttributes attributes, uint creation, uint flags, IntPtr template);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr CreateJobObjectW(IntPtr attributes, string name);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool SetInformationJobObject(IntPtr job, int kind,
            ref ExtendedLimits limits, uint length);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool AssignProcessToJobObject(IntPtr job, IntPtr process);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern uint ResumeThread(IntPtr thread);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool TerminateProcess(IntPtr process, uint code);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern uint WaitForSingleObject(IntPtr handle, uint milliseconds);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool GetExitCodeProcess(IntPtr process, out uint code);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool CloseHandle(IntPtr handle);

        private static Exception Error(string operation)
        {
            return new Win32Exception(Marshal.GetLastWin32Error(), operation);
        }

        private static string ObjectName(IntPtr handle)
        {
            uint needed;
            GetUserObjectInformationW(handle, 2, null, 0, out needed); // UOI_NAME
            var buffer = new StringBuilder((int)(needed / 2) + 1);
            if (!GetUserObjectInformationW(handle, 2, buffer, (uint)(buffer.Capacity * 2), out needed))
                throw Error("Read desktop/window-station name");
            return buffer.ToString();
        }

        public static string InputDesktopName()
        {
            IntPtr handle = OpenInputDesktop(0, false, 1); // DESKTOP_READOBJECTS
            if (handle == IntPtr.Zero) throw Error("Open input desktop");
            try { return ObjectName(handle); }
            finally { CloseDesktop(handle); }
        }

        private static IntPtr LogHandle(string path, bool input)
        {
            var attributes = new SecurityAttributes();
            attributes.length = Marshal.SizeOf(typeof(SecurityAttributes));
            attributes.inherit = true;
            IntPtr handle = CreateFileW(path, input ? 0x80000000u : 0x40000000u, 3,
                ref attributes, input ? 3u : 2u, 0x80, IntPtr.Zero);
            if (handle == new IntPtr(-1)) throw Error("Open test log/input: " + path);
            return handle;
        }

        // A null desktopName is ONLY for the caller's explicit -Visible opt-in.
        public static GuiTestProcess Start(string executable, string arguments, string cwd,
            string desktopName, string stdoutPath, string stderrPath)
        {
            var result = new GuiTestProcess();
            IntPtr output = IntPtr.Zero, error = IntPtr.Zero, input = IntPtr.Zero;
            ProcessInfo info = new ProcessInfo();
            try
            {
                if (!String.IsNullOrEmpty(desktopName))
                {
                    if (desktopName.IndexOfAny(new char[] {'\\', '/', '"'}) >= 0)
                        throw new ArgumentException("Use a plain desktop name");
                    if (!String.Equals(ObjectName(GetProcessWindowStation()), "WinSta0", StringComparison.OrdinalIgnoreCase))
                        throw new InvalidOperationException("Hidden GUI tests require the logged-in WinSta0 session.");
                    result.desktop = CreateDesktopW(desktopName, IntPtr.Zero, IntPtr.Zero, 0,
                        0x10000000, IntPtr.Zero); // GENERIC_ALL; never call SwitchDesktop
                    if (result.desktop == IntPtr.Zero) throw Error("Create hidden test desktop");
                    result.DesktopName = "WinSta0\\" + desktopName;
                }
                result.job = CreateJobObjectW(IntPtr.Zero, null);
                if (result.job == IntPtr.Zero) throw Error("Create test process job");
                var limits = new ExtendedLimits();
                limits.basic.flags = 0x2000; // JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
                if (!SetInformationJobObject(result.job, 9, ref limits, (uint)Marshal.SizeOf(typeof(ExtendedLimits))))
                    throw Error("Configure test process job");
                output = LogHandle(stdoutPath, false);
                error = LogHandle(stderrPath, false);
                input = LogHandle("NUL", true);
                var startup = new StartupInfo();
                startup.cb = Marshal.SizeOf(typeof(StartupInfo));
                startup.desktop = result.DesktopName;
                startup.flags = 0x100 | 0x80; // STARTF_USESTDHANDLES | STARTF_FORCEOFFFEEDBACK
                startup.stdin = input;
                startup.stdout = output;
                startup.stderr = error;
                var command = new StringBuilder("\"" + executable + "\" " + arguments);
                // Suspend until job assignment: even early children belong to our cleanup scope.
                if (!CreateProcessW(executable, command, IntPtr.Zero, IntPtr.Zero, true,
                        0x4 | 0x400 | 0x08000000, IntPtr.Zero, cwd, ref startup, out info))
                    throw Error("Launch disposable graphical test process");
                result.process = info.process;
                result.ProcessId = info.processId;
                if (!AssignProcessToJobObject(result.job, result.process))
                    throw Error("Assign test process to cleanup job");
                if (ResumeThread(info.thread) == UInt32.MaxValue)
                    throw Error("Resume test process");
                return result;
            }
            catch
            {
                if (result.process != IntPtr.Zero) TerminateProcess(result.process, 1);
                result.Dispose();
                throw;
            }
            finally
            {
                if (info.thread != IntPtr.Zero) CloseHandle(info.thread);
                if (output != IntPtr.Zero) CloseHandle(output);
                if (error != IntPtr.Zero) CloseHandle(error);
                if (input != IntPtr.Zero) CloseHandle(input);
            }
        }

        public bool WaitForExit(int milliseconds)
        {
            uint status = WaitForSingleObject(process, (uint)milliseconds);
            if (status == 0) return true;
            if (status == 258) return false;
            throw Error("Wait for GUI test process");
        }

        public uint ExitCode
        {
            get
            {
                uint code;
                if (!GetExitCodeProcess(process, out code)) throw Error("Read GUI test exit code");
                return code;
            }
        }

        public void Dispose()
        {
            // Job close kills this test tree, not any pre-existing artist process.
            if (job != IntPtr.Zero) { CloseHandle(job); job = IntPtr.Zero; }
            if (process != IntPtr.Zero) { CloseHandle(process); process = IntPtr.Zero; }
            if (desktop != IntPtr.Zero) { CloseDesktop(desktop); desktop = IntPtr.Zero; }
        }
    }
}
