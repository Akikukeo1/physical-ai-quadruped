"""Wait for an explicit search task before starting the simulator."""

import json
import math
import time
from pathlib import Path

from pai_motion.planner import ROOT, _codex_json


def interpret_initial(message, history, timeout=60, *, points=None, start=None):
    instructions = (
        "You are a simulated robot assistant using Codex CLI. Answer the user in Japanese. "
        "You can search for blue or green targets, return home, reobserve sightings and allow or avoid named areas. "
        "Do not claim you can inspect a named area as a task, use a camera, or control a physical robot. "
        "Return search_request only if the user explicitly requests searching for a blue or green target"
        + (" or visiting an available named point; " if points else "; ")
        + "otherwise use null and ask for a concrete search task when appropriate. Never invent a task or target. "
        "If the message combines a task and question, answer the question and include the task. "
        "search_request must preserve the user's task and constraints in Japanese. "
        "A search will start only after this response; do not claim it has already happened. "
        f"Dialogue: {json.dumps(history, ensure_ascii=False)}. Latest message: {message!r}"
    )
    if points:
        instructions += (
            " The user may also request visiting named points without searching for a color. "
            "Treat that as a valid task and place the full request in search_request. "
            f"Available point IDs and coordinates: {json.dumps(points, ensure_ascii=False)}."
        )
        if start is not None:
            distances = [{"id": p["id"], "distance_m": math.dist(start, p["position"])} for p in points]
            minimum = min(entry["distance_m"] for entry in distances)
            closest = [entry["id"] for entry in distances if math.isclose(entry["distance_m"], minimum, abs_tol=1e-9)]
            instructions += (
                f" Initial robot position: {json.dumps(start)}. Straight-line distances: {json.dumps(distances)}. "
                f"Nearest point IDs by straight-line distance: {json.dumps(closest)}. "
                "If the user asks for the nearest point, use the minimum straight-line distance, include its ID in "
                "search_request and explain that obstacles and actual route distance are unknown. "
                "If multiple points are equally nearest, ask which one to use."
            )
    value = _codex_json(instructions, ROOT / "schema/initial-dialogue.schema.json", timeout)
    if not isinstance(value, dict) or set(value) != {"reply", "search_request"}:
        raise ValueError("Invalid initial dialogue response")
    reply, request = value["reply"], value["search_request"]
    if not isinstance(reply, str) or not 1 <= len(reply.strip()) <= 300 or "\n" in reply:
        raise ValueError("Invalid initial dialogue reply")
    if request is not None and (not isinstance(request, str) or not 1 <= len(request.strip()) <= 500):
        raise ValueError("Invalid initial search request")
    return reply, request


def wait_for_search(message, path: Path, *, history=None, interpret=interpret_initial):
    history = [] if history is None else history
    last_id = None
    for index in range(12):
        reply, request = interpret(message, history)
        history.extend([{"role": "user", "text": message}, {"role": "assistant", "text": reply}])
        print(f"REPLY: {reply}", flush=True)
        if request is not None:
            path.write_text("", encoding="utf-8")
            return request
        if index == 11:
            break
        print("INPUT READY: 探索の指示を待っています。", flush=True)
        while True:
            if path.exists() and path.stat().st_size:
                if path.stat().st_size > 4096:
                    raise ValueError("Correction file exceeds 4096 bytes")
                value = json.loads(path.read_text(encoding="utf-8"))
                if not isinstance(value, dict) or set(value) != {"id", "message"}:
                    raise ValueError("Invalid dialogue message")
                event_id, text = value["id"], value["message"]
                if not isinstance(event_id, str) or not 1 <= len(event_id) <= 80:
                    raise ValueError("Invalid dialogue message ID")
                if not isinstance(text, str) or not 1 <= len(text.strip()) <= 500:
                    raise ValueError("Invalid dialogue message text")
                if event_id != last_id:
                    last_id, message = event_id, text
                    print(f"DIALOGUE INPUT: {message}", flush=True)
                    break
            time.sleep(0.2)
    raise ValueError("At most 12 messages before the search")
