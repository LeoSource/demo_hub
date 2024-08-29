import torch
from torch import nn
from torch.utils.data import random_split



def train(net,datasets,train_ratio,max_epochs):
    x_torch,y_torch = datasets[:]
    train_size = int(train_ratio*len(datasets))
    test_size = len(datasets) - train_size
    train_dataset,test_dataset = random_split(datasets,
                                            [train_size,test_size],
                                            generator=torch.Generator().manual_seed(42))

    criterion = nn.MSELoss()
    optimizer = torch.optim.LBFGS(net.parameters(), lr=0.1, history_size=200,
                                  tolerance_grad=1e-10, tolerance_change=1e-10, max_iter=50)
    patience = 10
    best_val_loss = float('inf')
    patience_counter = 0
    for epoch in range(max_epochs):
        net.train()
        x,y = train_dataset[:]
        def closure():
            optimizer.zero_grad()
            y_hat = net(x)
            loss = criterion(y_hat,y)
            loss.backward()
            return loss
        optimizer.step(closure)

        net.eval()
        val_loss = 0
        with torch.no_grad():
            x,y = test_dataset[:]
            y_hat = net(x)
            loss = criterion(y_hat,y)
            val_loss = loss.item()

        err = criterion(net(x_torch),y_torch)
        if (epoch+1) % 10 == 0:
            print('Epoch [{}/{}], Loss: {:.10f}'.format(epoch+1, max_epochs, err))

        if val_loss<best_val_loss:
            best_val_loss = val_loss
            patience_counter = 0
        else:
            patience_counter += 1
            print(f"Validation loss did not improve. Patience counter: {patience_counter}/{patience}")

        if patience_counter>=patience:
            print(f"Early stopping. Best val loss: {best_val_loss}")
            break

