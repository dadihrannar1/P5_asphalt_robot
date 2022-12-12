## Function collection for loading model
from torch import device, nn, cat, load
from torchvision.transforms.functional import resize

##Class is used by UNET class
# Link to source: https://github.com/abmoRobotics/P5/blob/796fd9765d8d9c8c51a52b2478dc2f781e506d38/Vision/model/model.py#L6
class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(DoubleConv, self).__init__()
        self.conv = nn.Sequential(
            #nn.Dropout2d(p=0.2),
            nn.Conv2d(in_channels, out_channels, 3, 1, 1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            #nn.Dropout2d(p=0.2),
            nn.Conv2d(out_channels, out_channels, 3, 1, 1, bias=False),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.conv(x)


##Class framework for model structure
# Link to source: https://github.com/abmoRobotics/P5/blob/796fd9765d8d9c8c51a52b2478dc2f781e506d38/Vision/model/model.py#L23
class UNET(nn.Module):
    def __init__(
            self, in_channels=3, out_channels=1, features=[64, 128, 256, 512],#, 512],
    ):
        super(UNET, self).__init__()
        self.decoder = nn.ModuleList()
        self.encoder = nn.ModuleList()
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)

        # encoder part of UNET
        for feature in features:
            self.encoder.append(DoubleConv(in_channels, feature))
            in_channels = feature

        # decoder part of UNET
        for feature in reversed(features):
            self.decoder.append(
                nn.ConvTranspose2d(
                    feature*2, feature, kernel_size=2, stride=2,
                )
            )
            self.decoder.append(DoubleConv(feature*2, feature))

        self.bottleneck = DoubleConv(features[-1], features[-1]*2)
        self.final_conv = nn.Conv2d(features[0], out_channels, kernel_size=1)

    def forward(self, x):
        #print(x.is_cuda)
        skip_connections = []

        for down in self.encoder:
            x = down(x) # Apply double convulution
            skip_connections.append(x)  # save output for the decoder (Skip connection)
            x = self.pool(x)    # Apply max pooling operation
        
        x = self.bottleneck(x)  # Apply double convulution for the bottleneck
        skip_connections = skip_connections[::-1]   # flip the order of the saved skip connections

        for idx in range(0, len(self.decoder), 2):
            x = self.decoder[idx](x) # Apply decoderampling
            skip_connection = skip_connections[idx//2]  # get skip connection

            if x.shape != skip_connection.shape:
                x = resize(x, size=skip_connection.shape[2:])

            concat_skip = cat((skip_connection, x), dim=1)    # concatenate skip connection
            x = self.decoder[idx+1](concat_skip)    # decoderample 

        return self.final_conv(x)


##Function to load model given a specific path
# Adapted to map processing to CPU
# Link to original: https://github.com/abmoRobotics/P5/blob/796fd9765d8d9c8c51a52b2478dc2f781e506d38/Vision/utils/utils.py#L141
def load_model(model_path, features=[64,128,256,512]):
    model = UNET(in_channels=3, out_channels=1, features=features)
    checkpoint = load(model_path, map_location=device('cpu'))
    model.load_state_dict(checkpoint)
    model.eval()
    model.cpu()

    return model