"""PAID: exactly one tiny model turn + fake tool. Requires explicit user approval."""
import argparse
import json
from pathlib import Path
import queue
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python3.13libs"))
from loom.provider import CodexTransport, initialize_params, thread_params


def response(transport, request_id):
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        try:
            event = transport.events.get(timeout=0.5)
        except queue.Empty:
            continue
        if event.get("transport_error"):
            raise RuntimeError(event["transport_error"])
        if event.get("id") == request_id:
            if "error" in event:
                raise RuntimeError(str(event["error"]))
            return event["result"]
        if "id" in event and "method" in event:
            transport.reject(event["id"], "Not supported")
    raise TimeoutError("No provider response")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-paid-provider", action="store_true")
    if not parser.parse_args().allow_paid_provider:
        parser.error("Obtain user approval and pass --allow-paid-provider. Consumes subscription usage.")
    transport = CodexTransport()
    try:
        transport.start()
        response(transport, transport.request("initialize", initialize_params()))
        transport.notify("initialized")
        account = response(transport, transport.request("account/read", {"refreshToken": False}))
        if (account.get("account") or {}).get("type") != "chatgpt":
            raise RuntimeError("Sign in with ChatGPT in Loom first")
        params = thread_params()
        params.update(ephemeral=True, baseInstructions="Call loom_smoke_echo once. Reply with its marker only.",
                      dynamicTools=[{"name": "loom_smoke_echo", "description": "Return a test marker.",
                                     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}}])
        thread_id = response(transport, transport.request("thread/start", params))["thread"]["id"]
        marker = "LOOM_OK_" + uuid.uuid4().hex[:8]
        transport.request("turn/start", {"threadId": thread_id, "input": [
            {"type": "text", "text": "Call loom_smoke_echo, then reply with its marker only."}]})
        deadline, calls, answer = time.monotonic() + 60, 0, ""
        while time.monotonic() < deadline:
            try:
                event = transport.events.get(timeout=0.5)
            except queue.Empty:
                continue
            if event.get("error") or event.get("transport_error"):
                raise RuntimeError(str(event))
            method, params = event.get("method"), event.get("params", {})
            if method == "item/tool/call":
                calls += 1
                if params["tool"] != "loom_smoke_echo" or calls > 1:
                    raise RuntimeError("Unexpected extra tool call")
                transport.reply(event["id"], {"success": True, "contentItems": [
                    {"type": "inputText", "text": json.dumps({"marker": marker})}]})
            elif "id" in event and method:
                transport.reject(event["id"], "Not supported")
            elif method == "item/agentMessage/delta":
                answer += params.get("delta", "")
            elif method == "turn/completed":
                assert params["turn"]["status"] == "completed", params["turn"]
                assert calls == 1 and marker in answer, "Missing tool round trip: " + answer
                print("PASS: PAID minimal real-provider tool round trip (one turn, one call)")
                return
        raise TimeoutError("Model turn timed out")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
