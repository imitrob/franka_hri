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

    def probabilistic_predict(self, 
            probabilistic_inputs, 
            role_description: str,
            max_new_tokens: int = 1000, 
            temperature: float = 0.0, 
            top_p: float = 1.0,
            repetition_penalty: float = 1.1,
            *args, **kwargs,
            ):
        # Step 1: Build the text prompt via chat template
        messages = [
            {"role": "system", "content": role_description},
            {"role": "user", "content": "<|start_of_text|>What is your faviourite food?"}  # Placeholder for user input
        ]
        text_prompt = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        prompt_ids = self.tokenizer(text_prompt, return_tensors="pt").input_ids.to(self.model.device)
        embedding_layer = self.model.get_input_embeddings()
        prompt_embeds = embedding_layer(prompt_ids)  # Shape: [1, prompt_seq_len, emb_dim]
        
        # Step 2: Create soft embeddings from probabilistic_inputs
        soft_embeddings = []
        for token_candidates in probabilistic_inputs:
            combined_embedding = None
            for token_str, weight in token_candidates.items():
                # Tokenize the candidate token (may produce multiple subword tokens)
                token_ids = self.tokenizer(token_str, add_special_tokens=False)["input_ids"]

                # Compute the weighted average of embeddings for all subword tokens
                token_embs = []
                for token_id in token_ids:
                    token_tensor = torch.tensor(token_id).to(self.model.device)
                    token_emb = embedding_layer(token_tensor)  # Shape: [1, emb_dim]
                    token_embs.append(token_emb)
                
                # Average the embeddings of all subword tokens
                avg_token_emb = torch.mean(torch.stack(token_embs, dim=0), dim=0)  # Shape: [1, emb_dim]
                weighted_emb = weight * avg_token_emb
                
                # Accumulate weighted embeddings
                if combined_embedding is None:
                    combined_embedding = weighted_emb
                else:
                    combined_embedding += weighted_emb
            
            # Remove the extra dimension and add to list
            soft_embeddings.append(combined_embedding.squeeze(0))  # Shape: [emb_dim]

        # Stack and add batch dimension
        soft_embeds = torch.stack(soft_embeddings, dim=0).unsqueeze(0)  # Shape: [1, num_soft_tokens, emb_dim]
        
        # Step 3: Find exact user content position
        # Generate the full prompt with chat template
        text_prompt = self.tokenizer.apply_chat_template(
            messages,
            tokenize=False,
            add_generation_prompt=True
        )
        # Tokenize the full prompt to get token IDs
        full_prompt_ids = self.tokenizer(text_prompt, return_tensors="pt")["input_ids"][0].tolist()

        
        # Locate the start of the user content
        found_replace_option = False
        for replace_option in [
                ["<|user|>", "<|assistant|>",0],
                ["<|start_of_text|>", "<|end_of_text|>",1],
                ["[|user|]", "[|assistant|]",2],
                ["<｜User｜>", "<｜Assistant｜>",3],
            ]:
            # Tokenize the user role token (<|user|>) and user message separately
            user_role_token = replace_option[0] #"<|start_of_text|>" #"<|user|>"
            user_role_token_ids = self.tokenizer(user_role_token, add_special_tokens=False)["input_ids"]

            try:
                # Find the position of the user role token
                user_start = None
                for i in range(len(full_prompt_ids) - len(user_role_token_ids) + 1):
                    if full_prompt_ids[i:i + len(user_role_token_ids)] == user_role_token_ids:
                        user_start = i + len(user_role_token_ids)  # Start of user content
                        break
                if user_start is None:
                    raise ValueError("Could not locate user role token in the prompt.")
                
                # Find the end of the user content (assume it ends at the assistant role token)
                assistant_role_token = replace_option[1] #"<|end_of_text|>" #"<|assistant|>"
                assistant_role_token_ids = self.tokenizer(assistant_role_token, add_special_tokens=False)["input_ids"]
                user_end = None
                for i in range(user_start, len(full_prompt_ids) - len(assistant_role_token_ids) + 1):
                    if full_prompt_ids[i:i + len(assistant_role_token_ids)] == assistant_role_token_ids:
                        user_end = i  # End of user content
                        break
                if user_end is None:
                    found_replace_option = False
                    continue
            except Exception as e:
                found_replace_option = False
                continue
            found_replace_option = True
            break
        if not found_replace_option:
            raise ValueError(f"Could not locate assistant role token in the prompt. {text_prompt}")
        
        # Step 4: Replace user content with soft embeddings
        combined_inputs_embeds = torch.cat([
            prompt_embeds[:, :user_start, :],  # System prompt and user role token
            soft_embeds,  # User's probabilistic input
            prompt_embeds[:, user_end:, :]  # Assistant role token and generation prompt
        ], dim=1)


        # Step 4: Create attention mask
        combined_attention_mask = torch.ones(combined_inputs_embeds.shape[:2], device=combined_inputs_embeds.device)
        
        # Step 5: Generate
        printer = StoppingCriteriaList([ProgressStoppingCriteria(total_steps=max_new_tokens)])
        generated_ids = self.model.generate(
            inputs_embeds=combined_inputs_embeds,
            attention_mask=combined_attention_mask,
            max_new_tokens=max_new_tokens,
            temperature=temperature,
            top_p=top_p,
            repetition_penalty=repetition_penalty,
            eos_token_id=self.tokenizer.eos_token_id,
            do_sample=False,
            stopping_criteria=printer,
        )

        generated_sequence = generated_ids[0]
        response = self.tokenizer.decode(generated_sequence, skip_special_tokens=True)
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
