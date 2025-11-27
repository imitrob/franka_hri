from transformers import AutoModelForCausalLM, AutoTokenizer, StoppingCriteria, StoppingCriteriaList
import torch
from transformers import BitsAndBytesConfig
from tqdm import tqdm

class ProgressStoppingCriteria(StoppingCriteria):
    def __init__(self, total_steps):
        self.pbar = tqdm(total=total_steps)
    def __call__(self, input_ids: torch.LongTensor, scores: torch.FloatTensor, **kwargs) -> bool:
        self.pbar.update(1)
        return False  # Never stop early, just track progress

QUANTIZATION = 16 # quickest

class SentenceProcessor():
    def __init__(self, model_name: str = "SultanR/SmolTulu-1.7b-Instruct"):
        """Good models for instruct:
            model_name = Qwen/Qwen2.5-0.5B-Instruct (1GB VRAM)
            model_name = SultanR/SmolTulu-1.7b-Instruct (3.3GB VRAM)
            deepseek-ai/DeepSeek-R1-Distill-Qwen-7B

        Args:
            model_name (str, optional): _description_. Defaults to "SultanR/SmolTulu-1.7b-Instruct".
        """
        print(f"Starting 3/3 Init LLM", flush=True)
        print(f"Memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
        print(f"Memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")

        if QUANTIZATION == 32:
            quantization_config = None
            torch_dtype = torch.float32
        elif QUANTIZATION == 16:
            quantization_config = None
            torch_dtype = torch.float16
        elif QUANTIZATION == 8:
            quantization_config = BitsAndBytesConfig(load_in_8bit=True)
            torch_dtype = torch.float32
        elif QUANTIZATION == 4:
            quantization_config = BitsAndBytesConfig(load_in_4bit=True)
            torch_dtype = torch.float32

        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype=torch_dtype,
            device_map="cuda", # needs to be fully loaded into the GPU or it's too slow!
            trust_remote_code=True,
            quantization_config=quantization_config
        )
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        print(f"3/3 Inited LLM", flush=True)
        print(f"Memory allocated: {torch.cuda.memory_allocated() / 1e9:.2f} GB")
        print(f"Memory reserved: {torch.cuda.memory_reserved() / 1e9:.2f} GB")


    def delete(self):
        self.model.to("cpu")
        del self.model

    def predict(self, 
                prompt: str, # input sentence is string
                role_description: str,
                max_new_tokens: int = 50, 
                temperature: float = 0.0, 
                top_p: float = 1.0,
                repetition_penalty: float = 1.1,
            ) -> dict:
        """ Returns as "parsed" instruct dict in format:
            Dict[str, str]: keys are always ("target_action", "target_object", "target_storage")
        """
        response = self.raw_predict(prompt, role_description, max_new_tokens, temperature, top_p, repetition_penalty)
        print(response, flush=True)

        response = response.replace(", ", ",")
        response = response.replace("'", "")
        response = response.replace("a can", "can")
        response_list = response.split(",")
        r = {}
        k_prev = ""
        for i in range(len(response_list)):
            s = self.remove_article(response_list[i]) # get rid of a, the..
            
            k, v = self.sort_types(s) # get rid of "action: ..."
            if k_prev == k: # order from model is: object, object2 right after each other; color, color2
                r[k+"2"] = v
            else:
                r[k] = v
            k_prev = k
        return r

    def raw_predict(self, 
                    prompt: str, 
                    role_description: str,
                    max_new_tokens: int = 1000, 
                    temperature: float = 0.0, 
                    top_p: float = 1.0,
                    repetition_penalty: float = 1.1,
                    *args, **kwargs,
                    ) -> str:
        """ Returns string output from LM. """
        messages = [
            {
            "role": "system",
            "content": role_description,
            },
            {"role": "user", "content": prompt}
        ]
        text = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        model_inputs = self.tokenizer([text], return_tensors="pt").to(self.model.device)

        printer = StoppingCriteriaList([ProgressStoppingCriteria(total_steps=max_new_tokens)])
        generated_ids = self.model.generate(
            **model_inputs,
            max_new_tokens=max_new_tokens,  # Allow space for full format
            temperature=temperature,
            top_p=top_p,  # Use full distribution
            repetition_penalty=repetition_penalty,
            eos_token_id=self.tokenizer.eos_token_id,
            do_sample=False,  # Force greedy decoding
            stopping_criteria=printer,
        )
        # Decode only the new tokens
        response = self.tokenizer.decode(
            generated_ids[0][model_inputs.input_ids.shape[-1]:],
            skip_special_tokens=True
        )

        return response


    def remove_article(self, str):
        if str[0:2] == "a ":
            str = str.replace("a ", "")
        if str[0:4] == "the ":
            str = str.replace("the ", "")
        return str

    COLORS = ["green", "blue", "red", "pink"]
    def remove_color(self, str):
        ''' Sometimes, model puts color to object, this is a workaround '''
        for color in self.COLORS:
            if color in str:
                str = str.replace(color+" ", "") # "blue box" -> "box"
                str = str.replace(color, "") # "blue" -> "", does nothing if not found
        return str
    
    RELATIONS = ["into"]
    def remove_relation(self, str):
        ''' Sometimes, model puts relation into an action, this is a workaround '''
        for relation in self.RELATIONS:
            if relation in str:
                str = str.replace(" "+relation, "") # "blue box" -> "box"
                str = str.replace(relation, "") # "blue" -> "", does nothing if not found
        return str

    def sort_types(self, str):
        if "action: " in str:
            str = str.split("action: ")[-1]
            str = self.remove_relation(str)
            return "target_action", str
        if "object: " in str:
            str = str.split("object: ")[-1]
            str = self.remove_color(str)
            return "target_object", str
        if "color: " in str:
            str = str.split("color: ")[-1]
            return "target_object_color", str
        if "relationship: " in str:
            str = str.split("relationship: ")[-1]
            return "relationship", str
        raise Exception(f"Either 'action:', 'object:', 'color: ' or 'relationship': in string {str}")


def main():
    sp = SentenceProcessor()
    # print(f"Result: {sp.raw_predict('Pick a green book.')}")
    
    output = sp.predict_with_probs(
        asr_prompt=[
            [0.0, {"Pick": 1.0, "Kick": 0.2}],
            [0.1, {"a": 0.9, "the": 0.1}],
            [0.2, {"blue": 0.8, "green": 0.2}],
            [0.3, {"box": 0.7, "blocks": 0.3}]
        ]
    )
    print(f"Result: {output}")
    # Returns: "Pick a blue box"

    try:
        while True:
            prompt = input("Enter: ")
            # print(f"Sample prompt: {prompt}")
            print(f"Result: {sp.raw_predict(prompt)}")
    except KeyboardInterrupt:
        exit()

if __name__ == "__main__":
    main()
