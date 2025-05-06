import torch
import torch.nn as nn
from transformers import ViTForImageClassification, ViTFeatureExtractor
import numpy as np

# Dummy dataset
def generate_dummy_dataset(num_samples=100):
    images = np.random.rand(num_samples, 3, 224, 224)  # [batch, channels, height, width]
    labels = np.random.randint(0, 5, num_samples)  # 5 item classes
    return images, labels

# Fine-tuning function
def finetune_vit():
    # Load pre-trained ViT
    model = ViTForImageClassification.from_pretrained('google/vit-base-patch16-224', num_labels=5)
    feature_extractor = ViTFeatureExtractor.from_pretrained('google/vit-base-patch16-224')
    
    # Dummy data
    images, labels = generate_dummy_dataset()
    
    # Training setup
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-5)
    loss_fn = nn.CrossEntropyLoss()
    
    # Fine-tune for 5 epochs
    model.train()
    for epoch in range(5):
        total_loss = 0
        for i in range(len(images)):
            img = torch.tensor(images[i], dtype=torch.float32)
            label = torch.tensor(labels[i], dtype=torch.long)
            
            # Forward pass
            outputs = model(pixel_values=img.unsqueeze(0))
            loss = loss_fn(outputs.logits, label.unsqueeze(0))
            
            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
        
        print(f"Epoch {epoch+1}, Loss: {total_loss/len(images)}")
    
    # Save model
    model.save_pretrained("vit_finetuned_warehouse")
    feature_extractor.save_pretrained("vit_finetuned_warehouse")

if __name__ == "__main__":
    finetune_vit()