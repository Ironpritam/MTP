import torch
import torch.nn as nn

# class GRURegressionModel(nn.Module):
#     def __init__(self, input_size, hidden_size, num_layers, output_size=4, dropout=0.2):
#         super(GRURegressionModel, self).__init__()
#         self.gru = nn.GRU(input_size, hidden_size, num_layers, batch_first=True, dropout=dropout)
#         self.attention = nn.Linear(hidden_size, 1)  # Attention layer
#         self.fc = nn.Linear(hidden_size, output_size)  # Fully connected layer
#         self.dropout = nn.Dropout(dropout)  # Additional dropout layer
#
#     def forward(self, x):
#         out, _ = self.gru(x)  # Shape: (batch_size, sequence_length, hidden_size)
#         attention_weights = torch.softmax(self.attention(out).squeeze(-1), dim=1)  # Shape: (batch_size, sequence_length)
#         context_vector = torch.sum(out * attention_weights.unsqueeze(-1), dim=1)  # Weighted sum
#         context_vector = self.dropout(context_vector)  # Apply dropout
#         out = self.fc(context_vector)  # Fully connected layer
#         return out  # Direct output without ReLU for regression



import torch.nn.functional as F

class GRUEncoder(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, dropout=0.2):
        super(GRUEncoder, self).__init__()
        self.gru = nn.GRU(input_size, hidden_size, num_layers, batch_first=True, dropout=dropout)

    def forward(self, x):
        outputs, hidden = self.gru(x)  # outputs: [batch_size, seq_len, hidden_size]
        return outputs, hidden


class Attention(nn.Module):
    def __init__(self, hidden_size):
        super(Attention, self).__init__()
        self.attn = nn.Linear(hidden_size, hidden_size)
        self.context = nn.Linear(hidden_size, 1, bias=False)

    def forward(self, encoder_outputs):
        scores = self.context(torch.tanh(self.attn(encoder_outputs)))  # [batch_size, seq_len, 1]
        attention_weights = F.softmax(scores, dim=1)  # [batch_size, seq_len, 1]
        context_vector = (encoder_outputs * attention_weights).sum(dim=1)  # [batch_size, hidden_size]
        return context_vector, attention_weights


class GRUDecoder(nn.Module):
    def __init__(self, hidden_size, output_size, dropout=0.2):
        super(GRUDecoder, self).__init__()
        self.fc1 = nn.Linear(hidden_size, hidden_size * 2)
        self.fc2 = nn.Linear(hidden_size * 2, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size // 2)
        self.fc4 = nn.Linear(hidden_size // 2, output_size)
        self.dropout = nn.Dropout(dropout)

    def forward(self, context_vector):
        x = F.leaky_relu(self.fc1(context_vector))
        x = self.dropout(x)
        x = F.leaky_relu(self.fc2(x))
        x = self.dropout(x)
        x = F.relu(self.fc3(x))
        x = self.dropout(x)
        output = self.fc4(x)  # Linear activation for regression
        return output


class GRURegressionModel(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, output_size, dropout=0.2):
        super(GRURegressionModel, self).__init__()
        self.encoder = GRUEncoder(input_size, hidden_size, num_layers, dropout)
        self.attention = Attention(hidden_size)
        self.decoder = GRUDecoder(hidden_size, output_size, dropout)

    def forward(self, x):
        encoder_outputs, _ = self.encoder(x)
        context_vector, attention_weights = self.attention(encoder_outputs)
        output = self.decoder(context_vector)
        return output, attention_weights
