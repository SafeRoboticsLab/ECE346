import torch
import numpy as np
from datetime import datetime
import os


class NeuralNetwork():
    def __init__(self, input_size: int, output_size: int, batch_size: int=64, lr: float = 1e-3, save_dir = None, load_model =None) -> None:
        
        '''
        This class implements a simple neural network with 3 layers of MLP.
        Parameters:
            input_size: The size of the input vector
            output_size: The size of the output vector
            save_dir: The directory to save the model
            load_model: The filepath to load the model 
        '''
        self.input_size = input_size
        self.output_size = output_size
        self.batch_size = batch_size
        self.save_dir = save_dir if save_dir is not None else "./"
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        
        self.model = torch.nn.Sequential(
            torch.nn.Linear(input_size, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, output_size),
        )
        
        self.loss_fn = torch.nn.MSELoss(reduction='mean')
        
        
        if load_model is not None:
            try:
                self.model.load_state_dict(torch.load(load_model))
                print(f"Model loaded from {load_model}")
            except Exception as e:
                print("Unable to load: ", e)
                
        self.model.to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)
    
    def train(self, x, y):
        x = torch.from_numpy(x).float().to(self.device).reshape(-1, self.batch_size, self.input_size)
        y = torch.from_numpy(y).float().to(self.device).reshape(-1, self.batch_size, self.output_size)

        num_b = x.shape[0]
        loss = 0
        for b in range(num_b):
            loss_batch = self.train_step(x[b,:,:], y[b,:,:])
            loss += loss_batch
        return loss/num_b

    def train_step(self, x, y):
        '''
        This function trains the model for one step
        Parameters:
            x: The input vector
            y: The target vector
        '''
        self.model.train()
        self.model.zero_grad()

        if isinstance(x, np.ndarray):
            x = torch.from_numpy(x).float().to(self.device)
            y = torch.from_numpy(y).float().reshape(-1, 1).to(self.device)
        
        y_pred = self.model(x).reshape(-1, self.output_size)
        
        assert y_pred.shape == y.shape, f"y_pred: {y_pred.shape}, y: {y.shape} are not equal"
        
        loss = self.loss_fn(y_pred, y)
        loss.backward()
        self.optimizer.step()
        
        return loss.item()
    
    def inference_step(self, x):
        '''
        This function predicts the output vector
        Parameters:
            x: The input vector
        '''
        with torch.no_grad():
            self.model.eval()
            x = torch.from_numpy(x).float().to(self.device)
            y_pred = self.model(x)
            return y_pred.detach().cpu().numpy()
        
    def save_model(self, filename = None):
        '''
        This function saves the model
        '''
        if filename is None:
            filename = "model_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".pt"
        path = os.path.join(self.save_dir, filename)
        torch.save(self.model.state_dict(), path)
        print("Model saved to ", path)
        