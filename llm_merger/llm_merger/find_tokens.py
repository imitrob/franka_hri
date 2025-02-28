from transformers import AutoTokenizer, AutoModel
import torch
import numpy as np

def get_semantic_similar_word(input_word, tokenizer, model, top_k=5):
    # Get vocabulary and filter single-token words
    vocab = tokenizer.get_vocab()
    single_token_words = [word for word in vocab.keys() 
                         if len(tokenizer.tokenize(word)) == 1]

    # Precompute embeddings for all single-token words
    device = model.device  # Use same device as model
    embedding_layer = model.get_input_embeddings()
    
    # Store normalized embeddings
    word_embeddings = {}
    for word in single_token_words:
        token_id = vocab[word]
        with torch.no_grad():
            embedding = embedding_layer(torch.tensor([token_id]).to(device))
        word_embeddings[word] = embedding.cpu().numpy()[0]
    
    # Normalize all embeddings
    for word in word_embeddings:
        word_embeddings[word] /= np.linalg.norm(word_embeddings[word])

    # Get input word embedding (handle multi-token words)
    input_ids = tokenizer.encode(input_word, add_special_tokens=False)
    if not input_ids:
        return None
        
    with torch.no_grad():
        input_embeds = embedding_layer(torch.tensor(input_ids).to(device)).cpu().numpy()
    
    # Average multi-token embeddings and normalize
    input_embedding = np.mean(input_embeds, axis=0)
    input_embedding /= np.linalg.norm(input_embedding)

    # Find closest embeddings
    similarities = []
    for word, emb in word_embeddings.items():
        sim = np.dot(input_embedding, emb)
        similarities.append((word, sim))

    # Return top k matches
    similarities.sort(key=lambda x: x[1], reverse=True)
    return [item[0] for item in similarities[:top_k]]



words = ["stop", "release", "home", "pick", "push", "pass", "point", "open", "close", "put", "place", "pick", "push", "pass", "place", "point", "open", "close", "put", "stop", "release", "home", "force", "to", "into", "onto", "from", "cup", "cube", "plate", "cup", "small", "medium", "large", "red", "green", "blue", "open", "closed", "table", "can", "box", "fork", "marker", "note", "flow", "transfer", "storage", "blade", "rack", "ledge", "stand", "platform",
         "fast", "slow", 
"large",
"big",
"small",
"short",
"long",
"high",
"low",
"heavy",
"light",
"new",
"old",
"strong",
"weak",
"cold",
"hot",
"bright",
"dark",
"clean",
"soft",
"hard",
"rough",
"smooth",
"warm",
         ]


model_names = [
    "SultanR/SmolTulu-1.7b-Instruct",
    "LGAI-EXAONE/EXAONE-3.5-2.4B-Instruct",
    "ibm-granite/granite-3.1-2b-instruct",
]

for model_name in model_names:
    print("======================")
    print(f"=== {model_name} ===")
    print("======================")
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    for word in words:
        token = tokenizer(word, add_special_tokens=False)["input_ids"]
        if len(token) != 1:
            print(word)
            # print(f"len: {len(token)}; Word {word} not exists to alternative")