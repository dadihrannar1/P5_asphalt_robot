import torch
import torchvision
from utils.dataset import CrackDataset
from torch.utils.data import DataLoader
from utils.model import UNET

def save_checkpoint(state, filename="TrainingModel.pth.tar"):
    print("=> Saving checkpoint")
    torch.save(state, filename)

def load_checkpoint(checkpoint, model):
    print("=> Loading checkpoint")
    model.load_state_dict(checkpoint["state_dict"])





# def get_testDS(
#     test_dir,
#     test_maskdir,
#     test_transform,
# ):

#     test_ds = CarvanaDataset(
#         image_dir=test_dir,
#         mask_dir=test_maskdir,
#         transform=test_transform,
#     )

#     test_loader = DataLoader(
#         test_ds,
#         batch_size=16,
#         num_workers=4,
#         pin_memory=True,
#         shuffle=True,
#     )

#     return test_loader

def get_loaders(
    train_dir,
    train_maskdir,
    val_dir,
    val_maskdir,
    batch_size,
    train_transform,
    val_transform,
    num_workers=4,
    pin_memory=True,
):
    train_ds = CrackDataset(
        image_dir=train_dir,
        mask_dir=train_maskdir,
        transform=train_transform,
    )

    train_loader = DataLoader(
        train_ds,
        batch_size=batch_size,
        num_workers=num_workers,
        pin_memory=pin_memory,
        shuffle=True,
    )

    val_ds = CrackDataset(
        image_dir=val_dir,
        mask_dir=val_maskdir,
        transform=val_transform,
    )

    val_loader = DataLoader(
        val_ds,
        batch_size=batch_size,
        num_workers=num_workers,
        pin_memory=pin_memory,
        shuffle=False,
    )



    return train_loader, val_loader

def check_accuracy(loader, model, device="cuda"):
    num_correct = 0
    num_pixels = 0
    dice_score = 0
    intersection = 0
    union = 0
    IoU = 0
    union1 = 0
    model.eval()

    with torch.no_grad():
        #print(type(loader))
        for x, y in loader:
            x = x.to(device)
            y = y.to(device).unsqueeze(1)
            preds = torch.sigmoid(model(x))
            preds = (preds > 0.5).float()
            num_correct += (preds == y).sum()
            
            num_pixels += torch.numel(preds)
            intersection += ((preds * y).sum())
            union1 +=(preds + y).sum()
            union = union1-intersection
            IoU = intersection / union
            dice_score += (2 * (preds * y).sum()) / (
                (preds + y).sum() + 1e-8
            )

    print(
        f"Got {num_correct}/{num_pixels} with acc {num_correct/num_pixels*100:.2f}"
    )
    print(f"Dice score: {dice_score/len(loader)}")
    print(f"IoU score: {IoU}")
    acc = num_correct/num_pixels

    model.train()
    return IoU, (dice_score/len(loader)), acc

def save_predictions_as_imgs(
    loader, model, folder="saved_images/", device="cuda"
):
    model.eval()
    for idx, (x, y) in enumerate(loader):
        x = x.to(device=device)
        with torch.no_grad():
            preds = torch.sigmoid(model(x))
            preds = (preds > 0.5).float()
        torchvision.utils.save_image(
            preds, f"{folder}/pred_{idx}.png"
        )
        torchvision.utils.save_image(y.unsqueeze(1), f"{folder}/{idx}.png")
        torchvision.utils.save_image(x, f"{folder}/original_{idx}.png")

    model.train()



def load_model(model_name, features=[64,128,256,512]):
    model = UNET(in_channels=3, out_channels=1, features=features)
    chk = torch.load(model_name)
    model.load_state_dict(chk['state_dict'], strict=False)
    model.eval()
    model.cuda()

    return model