"""LLM client for a vLLM server (OpenAI-compatible API).

Run:
    vllm serve LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct --port 8000

Then, point VLLM_BASE_URL to it (default: http://localhost:8000/v1).
"""
import json
import os
from openai import APIConnectionError, OpenAI

DEFAULT_BASE_URL = "http://localhost:8000/v1"
REQUEST_TIMEOUT = 120.0  # [s] generation of long reasoning chains can take a while

# Fixed sampling settings: deterministic, greedy decoding for reproducible extraction.
TEMPERATURE = 0.0
TOP_P = 1.0
REPETITION_PENALTY = 1.1
MAX_NEW_TOKENS = 1000


def _extract_json(text: str) -> dict:
    """Parse a JSON object from `text`. With guided decoding `text` is already
    pure JSON; this also tolerates a stray prefix/suffix by slicing to the
    outermost braces."""
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        if "{" in text and "}" in text:
            return json.loads(text[text.index("{"):text.rindex("}") + 1])
        raise


class SentenceProcessor():
    """Thin OpenAI-API caller, replaces the former in-process transformers model."""

    def __init__(self, base_url: str | None = None, api_key: str | None = None):
        """The model is whatever the vLLM server is serving; it is discovered
        from the server and exposed as `self.model_name`.

        Args:
            base_url (str, optional): vLLM server URL, defaults to $VLLM_BASE_URL or http://localhost:8000/v1
            api_key (str, optional): defaults to $VLLM_API_KEY or "EMPTY" (vLLM default)
        """
        self.client = OpenAI(
            base_url=base_url or os.environ.get("VLLM_BASE_URL", DEFAULT_BASE_URL),
            api_key=api_key or os.environ.get("VLLM_API_KEY", "EMPTY"),
            timeout=REQUEST_TIMEOUT,
        )
        # Fail fast with a helpful message when the server is down / serves nothing
        try:
            served = [m.id for m in self.client.models.list()]
        except APIConnectionError as e:
            raise RuntimeError(f"No vLLM server at {self.client.base_url}. Start one with: vllm serve <model>") from e
        if not served:
            raise RuntimeError(f"No model served at {self.client.base_url}. Start one with: vllm serve <model>")
        self.model_name = served[0]
        print(f"Connected to LLM server at {self.client.base_url}, model: {self.model_name}", flush=True)

    def raw_predict(self, prompt: str, role_description: str) -> str:
        """ Returns string output from LM. """
        completion = self.client.chat.completions.create(
            model=self.model_name,
            messages=[
                {"role": "system", "content": role_description},
                {"role": "user", "content": prompt},
            ],
            max_tokens=MAX_NEW_TOKENS,
            temperature=TEMPERATURE,
            top_p=TOP_P,
            extra_body={"repetition_penalty": REPETITION_PENALTY},  # vLLM sampling extension
        )
        return completion.choices[0].message.content

    def predict_structured(self, prompt: str, role_description: str, schema: dict) -> dict:
        """Constrained generation via the OpenAI-standard structured-outputs
        API (`response_format` with a json_schema). vLLM enforces the schema
        with its structured-output backend, so the reply is valid JSON with
        allowed values only (out-of-enum values are impossible). Returns the
        parsed dict.

        Note: the legacy `guided_json` in `extra_body` is silently ignored
        unless the server is launched with a guided-decoding backend, so we
        use `response_format`, which is enforced by default.

        `enable_thinking=False` turns off Qwen3-style reasoning so the answer
        lands in `message.content` (otherwise the <think> block is routed to
        `reasoning_content` and `content` comes back empty). The kwarg is
        ignored by chat templates that don't use it, so it is safe for all
        models."""
        completion = self.client.chat.completions.create(
            model=self.model_name,
            messages=[
                {"role": "system", "content": role_description},
                {"role": "user", "content": prompt},
            ],
            max_tokens=MAX_NEW_TOKENS,
            temperature=TEMPERATURE,
            top_p=TOP_P,
            response_format={
                "type": "json_schema",
                "json_schema": {"name": "skill_command", "schema": schema},
            },
            extra_body={
                "repetition_penalty": REPETITION_PENALTY,
                "chat_template_kwargs": {"enable_thinking": False},
            },
        )
        msg = completion.choices[0].message
        text = (msg.content or "").strip()
        if not text:  # reasoning models may leave content empty, answer in reasoning_content
            text = (getattr(msg, "reasoning_content", None) or "").strip()
        if not text:
            raise RuntimeError(
                f"Empty response from model '{self.model_name}'. Ensure the vLLM server "
                f"enforces guided decoding and thinking is disabled. finish_reason="
                f"{completion.choices[0].finish_reason}"
            )
        return _extract_json(text)

    def delete(self):
        self.client.close()


def main():
    """Manual check: `python llm.py` with a running vLLM server."""
    sp = SentenceProcessor()
    try:
        while True:
            prompt = input("Enter: ")
            print(f"Result: {sp.raw_predict(prompt, role_description='You are a helpful assistant.')}")
    except KeyboardInterrupt:
        exit()


if __name__ == "__main__":
    main()
