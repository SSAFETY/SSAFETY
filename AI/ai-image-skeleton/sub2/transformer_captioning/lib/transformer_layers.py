import torch
import torch.nn as nn
from torch.nn import functional as F
import math

############################################################################
# Req 2-2: PositionalEncoding 구현                                          #
############################################################################
class PositionalEncoding(nn.Module):
    """
    각 토큰들의 위치 정보를 인코딩해주는 역할을 합니다. Positional encoding 레이어에는 간단한
    sine, cosine 함수로 구현되어 있기 때문에 별도의 학습 가능한 파라미터는 없습니다.
    """

    def __init__(self, embed_dim, dropout=0.1, max_len=5000):
        """
        PositionalEncoding 레이어 구현.
        Inputs:
         - embed_dim: the size of the embed dimension
         - dropout: the dropout value
         - max_len: the maximum possible length of the incoming sequence
        """
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)
        assert embed_dim % 2 == 0
        # Create an array with a "batch dimension" of 1 (which will broadcast
        # across all examples in the batch).
        pe = torch.zeros(1, max_len, embed_dim)
        ############################################################################
        # TODO: Transformer_Captioning.ipynb에 서술된 positional encoding array 구현   #
        # 본 실습의 목표는 pe 매트릭스의 row를 exponent 0, 0, 2, 2, 4, 4, etc. 로 구성된      #
        # sine, cosine 함수를 번갈아 사용하여 embed_dim의 크기 까지로 구현하는 것입니다.          #
        # 전체 구현 코드는 5줄 정도 됩니다                                                 #
        ############################################################################
        # *****START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****

        pass

        # *****END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****
        ############################################################################
        #                             END OF YOUR CODE                             #
        ############################################################################

        # Positional encoding은 모델 파라미터와 함께 저장됩니다.
        self.register_buffer('pe', pe)

    def forward(self, x):
        """
        Element-wise로 입력 시퀀스에 positional embeddings을 더해줍니다.
        Inputs:
         - x: the sequence fed to the positional encoder model, of shape
              (N, S, D), where N is the batch size, S is the sequence length and
              D is embed dim
        Returns:
         - output: the input sequence + positional encodings, of shape (N, S, D)
        """
        N, S, D = x.shape
        # Create a placeholder, to be overwritten by your code below.
        output = torch.empty((N, S, D))
        ############################################################################
        # TODO: 입력 시퀀스의 동일한 index에 positional encoding을 더해줍니다.               #
        # positional encoding을 더한 뒤 dropout도 적용해주어야 합니다                       #
        ############################################################################
        # *****START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****

        pass

        # *****END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****
        ############################################################################
        #                             END OF YOUR CODE                             #
        ############################################################################
        return output

############################################################################
# Req 2-1: multi-head scaled dot-product attention 구현                     #
############################################################################
class MultiHeadAttention(nn.Module):
    """
    "Attention Is All You Need" (https://arxiv.org/abs/1706.03762)에서 소개된
    masked attention의 간단한 버전의 모델 layer입니다.
    Usage:
      attn = MultiHeadAttention(embed_dim, num_heads=2)
      # self-attention
      data = torch.randn(batch_size, sequence_length, embed_dim)
      self_attn_output = attn(query=data, key=data, value=data)
      # attention using two inputs
      other_data = torch.randn(batch_size, sequence_length, embed_dim)
      attn_output = attn(query=data, key=other_data, value=other_data)
    """

    def __init__(self, embed_dim, num_heads, dropout=0.1):
        """
        MultiHeadAttention layer를 정의합니.
        Inputs:
         - embed_dim: Dimension of the token embedding
         - num_heads: Number of attention heads
         - dropout: Dropout probability
        """
        super().__init__()
        assert embed_dim % num_heads == 0

        # 본 실습에서 사용한 key, query, value, proj 레이어의 정의입니다.
        # 실습 내의 random seed가 고정되어 있기 때문에, 아래 정의된 레이어의 순서를 바꾸면 안됩니다.
        self.key = nn.Linear(embed_dim, embed_dim)
        self.query = nn.Linear(embed_dim, embed_dim)
        self.value = nn.Linear(embed_dim, embed_dim)
        self.proj = nn.Linear(embed_dim, embed_dim)

        ############################################################################
        # TODO: 앞서 정의된 layer 이외에 Transformer_Captioning.ipynb를 참고하여 attention #
        # 연산 수행을 위한 레이어들을 정의합니다.                                            #
        # 또한 softmax step 이후에 dropout을 사용하기 때문에 해당 레이어 정의도 필요합니다.        #
        ############################################################################
        # *****START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****

        pass

        # *****END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****
        ############################################################################
        #                             END OF YOUR CODE                             #
        ############################################################################

    def forward(self, query, key, value, attn_mask=None):
        """
        제공된 data의 masked attention output을 연산합니다.
        이때 모든 attention head들은 병렬적으로 연산합니다.
        파라미터 shape에 대한 정의는 다음과 같습니다.
        N: batch size / S: source sequence length / T: target sequence length / E: embedding dimension.

        Inputs:
        - query: Input data to be used as the query, of shape (N, S, E)
        - key: Input data to be used as the key, of shape (N, T, E)
        - value: Input data to be used as the value, of shape (N, T, E)
        - attn_mask: Array of shape (T, S) where mask[i,j] == 0 indicates token
          i in the target should not be influenced by token j in the source.
        Returns:
        - output: Tensor of shape (N, S, E) giving the weighted combination of
          data in value according to the attention weights calculated using key
          and query.
        """
        N, S, D = query.shape
        N, T, D = value.shape
        # Create a placeholder, to be overwritten by your code below.
        output = torch.empty((N, T, D))

        ############################################################################
        # TODO: Transformer_Captioning.ipynb의 multiheaded attention 수식을 구현합니다. #
        # 힌트:                                                                     #
        #  1) Shape을 (N, T, E) 에서 (N, T, H, E/H)로 변환해야 합니다.                    #
        #     이때 H 는 head의 개수입니다.                                               #
        #  2) torch.matmul 병렬적으로 batch matrix multiply을 수행할 수 있습니다.           #
        #     https://pytorch.org/docs/stable/generated/torch.matmul.html          #
        #  3) value 값에 영향을 주지 않기 위해서 score가 어떤식으로 출력되어야 하는지 생각하여        #
        #     attn_mask를 적용해 봅니다.                                                #
        #     PyTorch의 masked_fill 함수를 참고합니다.                                   #
        ############################################################################
        # *****START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****

        pass

        # *****END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****
        ############################################################################
        #                             END OF YOUR CODE                             #
        ############################################################################
        return output