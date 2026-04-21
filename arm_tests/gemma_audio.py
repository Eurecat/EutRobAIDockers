from transformers import AutoProcessor, AutoModelForMultimodalLM
import time

MODEL_ID = "google/gemma-4-E2B-it"

# Load model
start_load = time.time()
processor = AutoProcessor.from_pretrained(MODEL_ID)
model = AutoModelForMultimodalLM.from_pretrained(
    MODEL_ID, 
    dtype="auto", 
    device_map="auto"
)
end_load = time.time()
load_time = end_load - start_load
print(f"Model loading time: {load_time:.2f} seconds")

# Prompt - add audio before text
messages = [
    {
        "role": "user",
        "content": [
            {"type": "audio", "audio": "harvard.wav"},
            {"type": "text", "text": "Transcribe the following speech segment in its original language. Follow these specific instructions for formatting the answer:\n* Only output the transcription, with no newlines.\n* When transcribing numbers, write the digits, i.e. write 1.7 and not one point seven, and write 3 instead of three."},
        ]
    }
]

# Process input
inputs = processor.apply_chat_template(
    messages,
    tokenize=True,
    return_dict=True,
    return_tensors="pt",
    add_generation_prompt=True,
).to(model.device)
input_len = inputs["input_ids"].shape[-1]

# Generate output
start_inference = time.time()
outputs = model.generate(**inputs, max_new_tokens=512)
end_inference = time.time()
inference_time = end_inference - start_inference
print(f"Inference time: {inference_time:.2f} seconds")
response = processor.decode(outputs[0][input_len:], skip_special_tokens=False)

# Parse output
processor.parse_response(response)

print(response)