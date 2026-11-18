"""Optional OpenAI ranking for locally validated recovery choices."""
import json
import os
from concurrent.futures import ThreadPoolExecutor
from urllib.request import Request, urlopen

DECISION_BUDGET = 1.5
WAIT_SECONDS = 1.0


def request_choice(key, model, candidates, evidence):
    ids = ["WAIT", *(candidate["id"] for candidate in candidates)]
    payload = {
        "model": model,
        "store": False,
        "max_output_tokens": 256,
        "reasoning": {"effort": "minimal"},
        "instructions": (
            "Select one prechecked maneuver for a simulated parking rover. "
            "Prefer the option that makes efficient progress toward the parking row. "
            "Choose WAIT if the options offer little clearance progress. "
            "Return only the selected ID in the schema."
        ),
        "input": json.dumps({"candidates": candidates, "observations": evidence}),
        "text": {"format": {
            "type": "json_schema", "name": "recovery_choice", "strict": True,
            "schema": {
                "type": "object", "properties": {"choice": {"type": "string", "enum": ids}},
                "required": ["choice"], "additionalProperties": False,
            },
        }},
    }
    request = Request(
        "https://api.openai.com/v1/responses", data=json.dumps(payload).encode(),
        headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"},
    )
    with urlopen(request, timeout=3.0) as response:
        result = json.load(response)
    if result.get("status") != "completed":
        return None
    for item in result.get("output", []):
        for part in item.get("content", []):
            if part.get("type") == "output_text":
                choice = json.loads(part["text"])["choice"]
                return choice if choice in ids else None
    return None


class LLMAdvisor:
    def __init__(self, key, model):
        self.key = key
        self.model = model
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pending = None

    @classmethod
    def from_environment(cls):
        key = os.environ.get("OPENAI_API_KEY")
        if os.environ.get("LOT10_LLM") != "1" or not key:
            return None
        return cls(key, os.environ.get("LOT10_LLM_MODEL", "gpt-5-mini"))

    def submit(self, candidates, evidence):
        if self.pending is not None and not self.pending.done():
            return None
        self.pending = self.worker.submit(request_choice, self.key, self.model,
                                          candidates, evidence)
        return self.pending

    def close(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
