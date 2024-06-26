# torchvision lib consists of popular datasets, model architectures,
# and common image transformations for computer vision.
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision

#################################################
# Dataset
# E.g.: MNIST
batch_size = 64
data_loader = torch.utils.data.DataLoader(
    torchvision.datasets.MNIST(),
    batch_size=batch_size, shuffle=True)

examples = enumerate(data_loader)
batch_idx, (example_data, example_label) = next(examples)

#################################################
# Neural Network Model
# Creating model


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 10, kernel_size=5)
        self.conv2 = nn.Conv2d(10, 20, kernel_size=5)
        self.conv2_drop = nn.Dropout2d()
        self.fc1 = nn.Linear(320, 50)
        self.fc2 = nn.Linear(50, 10)

    def forward(self, x):
        x = F.relu(F.max_pool2d(self.conv1(x), 2))
        x = F.relu(F.max_pool2d(self.conv2_drop(self.conv2(x)), 2))
        x = x.view(-1, 320)
        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        return F.log_softmax(x)


learning_rate = 0.01
momentum = 0.5
network = Net()
optimizer = optim.SGD(network.parameters(), lr=learning_rate,
                      momentum=momentum)

train_losses = []
train_counter = []
log_interval = 10


def train(epoch):
    network.train()
    for batch_idx, (data, target) in enumerate(data_loader):
        optimizer.zero_grad()
        output = network(data)
        loss = F.nll_loss(output, target)
        loss.backward()
        optimizer.step()
        if batch_idx % log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch, batch_idx * len(data), len(data_loader.dataset),
                100. * batch_idx / len(data_loader), loss.item()))
            train_losses.append(loss.item())
            train_counter.append(
                (batch_idx*64) + ((epoch-1)*len(data_loader.dataset)))

            network_path = os.path.join(os.getcwd(), 'results/model.pth')
            optimizer_path = os.path.join(os.getcwd(), 'results/optimizer.pth')
            torch.save(network.state_dict(), network_path)
            torch.save(optimizer.state_dict(), optimizer_path)


n_epochs = 3
test_losses = []
test_counter = [i*len(data_loader.dataset) for i in range(n_epochs + 1)]


def test():
    network.eval()
    test_loss = 0
    correct = 0
    with torch.no_grad():
        for data, target in data_loader:
            output = network(data)
            test_loss += F.nll_loss(output, target, size_average=False).item()
            pred = output.data.max(1, keepdim=True)[1]
            correct += pred.eq(target.data.view_as(pred)).sum()
    test_loss /= len(data_loader.dataset)
    test_losses.append(test_loss)
    print('\nTest set: Avg. loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
        test_loss, correct, len(data_loader.dataset),
        100. * correct / len(data_loader.dataset)))


# Loading models
continued_network = Net()
network_state_dict = torch.load('results/model.pth')
continued_network.load_state_dict(network_state_dict)
optimizer_state_dict = torch.load('results/optimizer.pth')
optimizer.load_state_dict(optimizer_state_dict)

