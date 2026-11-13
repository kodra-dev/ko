"""Exercise the real official app-server locally. Does not start a paid model turn.

--chat additionally makes one small subscription-backed tool round trip, after
sign-in in the panel. Its fake tool deliberately has no access to Houdini.
"""
import argparse
import json
from pathlib import Path
import queue
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python3.13libs"))
from ko_ai.provider import CodexTransport, initialize_params, thread_params


def wait_response(transport, request_id, timeout=40):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            event = transport.events.get(timeout=0.5)
        except queue.Empty:
            continue
        if event.get("transport_error"):
            raise RuntimeError(event["transport_error"] + "\n" + "\n".join(transport.stderr))
        if event.get("id") == request_id:
            if "error" in event:
                raise RuntimeError(json.dumps(event["error"]))
            return event["result"]
        if "id" in event and "method" in event:
            transport.reject(event["id"], "Not supported in smoke test")
    raise TimeoutError("No response from Codex: " + "\n".join(transport.stderr))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--chat", action="store_true")
    args = parser.parse_args()
    transport = CodexTransport()
    try:
        transport.start()
        result = wait_response(transport, transport.request("initialize", initialize_params()))
        transport.notify("initialized")
        print("PASS: initialize", result.get("userAgent", ""))
        account = wait_response(transport, transport.request("account/read", {"refreshToken": False}))
        kind = (account.get("account") or {}).get("type")
        print("PASS: account/read; auth type:", kind or "not signed in")
        models = wait_response(transport, transport.request("model/list", {}))
        print("PASS: model/list; models:", len(models.get("data", [])))
        params = thread_params()
        params["ephemeral"] = True
        thread = wait_response(transport, transport.request("thread/start", params))
        thread_id = thread["thread"]["id"]
        print("PASS: thread/start with dynamic Houdini tool schemas")
        if not args.chat:
            print("No model turn requested. To test actual inference, sign in in Ko AI, then rerun with --chat.")
            return
        if kind != "chatgpt":
            raise RuntimeError("Sign in with ChatGPT in Ko AI first")
        request_id = transport.request("turn/start", {"threadId": thread_id, "input": [{
            "type": "text", "text": "Call ko_scene operation=context, path=/, query='', offset=0, limit=1. "
            "Then respond with the exact smoke_marker value returned. Do not use any other tools."}]})
        deadline = time.monotonic() + 120
        called, answer = False, ""
        while time.monotonic() < deadline:
            try:
                event = transport.events.get(timeout=1)
            except queue.Empty:
                continue
            if event.get("error") or event.get("transport_error"):
                raise RuntimeError(str(event))
            method = event.get("method")
            params = event.get("params", {})
            if method == "item/tool/call":
                if params["tool"] != "ko_scene":
                    raise RuntimeError("Unexpected tool: " + params["tool"])
                called = True
                transport.reply(event["id"], {"success": True, "contentItems": [{
                    "type": "inputText", "text": '{"smoke_marker":"KO_AI_ROUNDTRIP_OK"}'}]})
            elif "id" in event and method:
                transport.reject(event["id"], "Not supported")
            elif method == "item/agentMessage/delta":
                answer += params.get("delta", "")
            elif method == "turn/completed":
                if params["turn"]["status"] != "completed":
                    raise RuntimeError(str(params["turn"]))
                if not called or "KO_AI_ROUNDTRIP_OK" not in answer:
                    raise RuntimeError("Tool round trip missing: " + answer)
                print("PASS: real ChatGPT subscription inference + dynamic tool round trip")
                return
        raise TimeoutError("Model turn did not complete")
    finally:
        transport.close()


if __name__ == "__main__":
    main()
