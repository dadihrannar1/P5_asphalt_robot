import torch
import albumentations as A
from albumentations.pytorch import ToTensorV2
from torch.nn.modules import loss
from tqdm import tqdm
import torch.nn as nn
import torch.optim as optim
import torch.utils
from utils.model import UNET
from torch.utils.tensorboard import SummaryWriter
import torchvision
from utils.utils import (
    load_checkpoint,
    save_checkpoint,
    get_loaders,
    check_accuracy,
    save_predictions_as_imgs,
)
#tensorboard
#writer = SummaryWriter('runs/fashion_mnist_experiment_1')

# Hyperparameters etc.
LEARNING_RATE = 1e-4     # original 1e-4
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
BATCH_SIZE = 1
NUM_EPOCHS = 90
NUM_WORKERS = 4
IMAGE_HEIGHT = int(1920)#160*2 1280 originally
IMAGE_WIDTH = int(1080)#240*2  1918 originally
PIN_MEMORY = True
LOAD_MODEL = False
TRAIN_IMG_DIR = "Training/Data/Train_data/RGB"
TRAIN_MASK_DIR = "Training/Data/Train_data/MASK" 
VAL_IMG_DIR = "Training/Data/Test_data/RGB"
VAL_MASK_DIR = "Training/Data/Test_data/MASK"


def train_fn(loader, model, optimizer, loss_fn, scaler):
    loop = tqdm(loader)
    max_score = 0
    total_loss = 0
    img_grid = 0
    img_grid2 = 0
    for batch_idx, (data, targets) in enumerate(loop):

        # img_grid = torchvision.utils.make_grid(data)
        # img_grid2 = torchvision.utils.make_grid(targets)

        data = data.to(device=DEVICE)
        targets = targets.float().unsqueeze(1).to(device=DEVICE)

        # forward
        with torch.cuda.amp.autocast():
            predictions = model(data)
            loss = loss_fn(predictions, targets)

        # backward
        optimizer.zero_grad()
        scaler.scale(loss).backward()
        scaler.step(optimizer)
        scaler.update()
        # update tqdm loop
        loop.set_postfix(loss=loss.item())
        total_loss += loss.item()
    # tb.add_image('image', img_grid)
    # tb.add_image('mask', img_grid2)
    return total_loss




def main():
    train_transform = A.Compose(
        [
            A.Resize(height=IMAGE_HEIGHT, width=IMAGE_WIDTH),
            A.Rotate(limit=35, p=1.0),
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.1),
            A.RandomBrightnessContrast(p=0.3), #Originally not used
            A.Normalize(
                mean=[0.0, 0.0, 0.0],
                std=[1.0, 1.0, 1.0],
                max_pixel_value=255.0,
            ),
            ToTensorV2(),
        ],
    )

    val_transforms = A.Compose(
        [
            A.Resize(height=IMAGE_HEIGHT, width=IMAGE_WIDTH),
            A.Normalize(
                mean=[0.0, 0.0, 0.0],
                std=[1.0, 1.0, 1.0],
                max_pixel_value=255.0,
            ),
            ToTensorV2(),
        ],
    )



    model = UNET(in_channels=3, out_channels=1, features=[32,64,128,256]).to(DEVICE)
    loss_fn = nn.BCEWithLogitsLoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    train_loader, val_loader = get_loaders(
        TRAIN_IMG_DIR,
        TRAIN_MASK_DIR,
        VAL_IMG_DIR,
        VAL_MASK_DIR,
        BATCH_SIZE,
        train_transform,
        val_transforms,
        NUM_WORKERS,
        PIN_MEMORY,
    )

    if LOAD_MODEL:
        load_checkpoint(torch.load("model/crack500BrightnessAugmentationv3.pth.tar"), model)


    check_accuracy(val_loader, model, device=DEVICE)
    scaler = torch.cuda.amp.GradScaler()
    max_score = 0

    tb = SummaryWriter()
    
    
    for epoch in range(0, NUM_EPOCHS):
        loss = 0
        loss = train_fn(train_loader, model, optimizer, loss_fn, scaler)

        # save model
        checkpoint = {
            "state_dict": model.state_dict(),
            "optimizer": optimizer.state_dict(),
        }
        

        # check accuracy
        IoU, F1, acc= check_accuracy(val_loader, model, device=DEVICE)
        
        if IoU > max_score:
            print("Best model found => saving")
            max_score = IoU
            # Save model
            save_checkpoint(checkpoint)

            # print some examples to a folder
            save_predictions_as_imgs(
            val_loader, model, folder="Training\Images", device=DEVICE
            )

        if epoch == 30:
            print("Changing learning rate to 1e-5")
            optimizer.param_groups[0]['lr'] = 1e-5
        if epoch == 60:
            print("Changing learning rate to 1e-6")
            optimizer.param_groups[0]['lr'] = 1e-6
        
        print(f"EPOCH: {epoch}/{NUM_EPOCHS}")
        tb.add_scalar('Accuraccy', acc, epoch)
        tb.add_scalar('Loss', loss/len(train_loader), epoch)
        tb.add_scalar('F1-score', F1, epoch)
        tb.add_scalar('IoU', IoU, epoch)
    tb.close()
def tester():

    train_transform = A.Compose(
        [
            A.Resize(height=IMAGE_HEIGHT, width=IMAGE_WIDTH),
            A.Rotate(limit=35, p=1.0),
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.1),
            A.RandomBrightness(p=0.3), #Originally not used
            A.Normalize(
                mean=[0.0, 0.0, 0.0],
                std=[1.0, 1.0, 1.0],
                max_pixel_value=255.0,
            ),
            ToTensorV2(),
        ],
    )

    val_transforms = A.Compose(
        [
            A.Resize(height=IMAGE_HEIGHT, width=IMAGE_WIDTH),
            A.Normalize(
                mean=[0.0, 0.0, 0.0],
                std=[1.0, 1.0, 1.0],
                max_pixel_value=255.0,
            ),
            ToTensorV2(),
        ],
    )

    train_loader, val_loader = get_loaders(
        TRAIN_IMG_DIR,
        TRAIN_MASK_DIR,
        VAL_IMG_DIR,
        VAL_MASK_DIR,
        BATCH_SIZE,
        train_transform,
        val_transforms,
        NUM_WORKERS,
        PIN_MEMORY,
    )
    tb = SummaryWriter()
    dataiter = iter(train_loader)
    images, labels = dataiter.next()

    # create grid of images
    img_grid = torchvision.utils.make_grid(images)

    # show images
   # matplotlib_imshow(img_grid, one_channel=True)
    #network = UNET()
    # write to tensorboard
    tb.add_image('four_fashion_mnist_images', img_grid)
    #tb.add_graph(network, images)
    tb.close()
            

if __name__ == "__main__":
    main()