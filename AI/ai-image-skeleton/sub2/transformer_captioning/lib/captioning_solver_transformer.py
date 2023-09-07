import numpy as np

from . import optim
from .coco_utils import sample_coco_minibatch, decode_captions

import torch


class CaptioningSolverTransformer(object):
    """
    CaptioningSolverTransformer 클래스에는 transformer 기반의 이미지 캡셔닝 모델을
    학습시키기 위해서 필요한 로직들을 모두 포함하고 있습니다.
    모델을 학습시키기 위해서는 먼저 모델, 데이터셋, 다양한 옵션(learning rate, batch size, etc)를
    전달하여 CaptionSolverTransformer instance를 생성해야 합니다.
    Instance를 생성한 이후 train() 메소드를 호출하여 학습을 진행할 수 있습니다.
    학습이 완료된 이후 solver 내의 loss_history에는 학습 시 기록했던 loss 값들이 저장되어 있습ㄴ디ㅏ.

    위의 process에 대한 예시 코드는 다음과 같습니다:
    data = load_coco_data()
    model = MyAwesomeTransformerModel(hidden_dim=100)
    solver = CaptioningSolver(model, data,
                    optim_config={
                      'learning_rate': 1e-3,
                    },
                    num_epochs=10, batch_size=100,
                    print_every=100)
    solver.train()


    API:
      Inputs:
      - features: Array giving a minibatch of features for images, of shape (N, D
      - captions: Array of captions for those images, of shape (N, T) where
        each element is in the range (0, V].
      Returns:
      - loss: Scalar giving the loss
      - grads: Dictionary with the same keys as self.params mapping parameter
        names to gradients of the loss with respect to those parameters.
    """

    def __init__(self, model, data, idx_to_word, **kwargs):
        """
        CaptioningSolver instance를 생성합니다.
        Required arguments:
        - model: A model object conforming to the API described above
        - data: A dictionary of training and validation data from load_coco_data
        Optional arguments:
        - learning_rate: Learning rate of optimizer.
        - batch_size: Size of minibatches used to compute loss and gradient during
          training.
        - num_epochs: The number of epochs to run for during training.
        - print_every: Integer; training losses will be printed every print_every
          iterations.
        - verbose: Boolean; if set to false then no output will be printed during
          training.
        """
        self.model = model
        self.data = data

        # Unpack keyword arguments
        self.learning_rate = kwargs.pop("learning_rate", 0.001)
        self.batch_size = kwargs.pop("batch_size", 100)
        self.num_epochs = kwargs.pop("num_epochs", 10)

        self.print_every = kwargs.pop("print_every", 10)
        self.verbose = kwargs.pop("verbose", True)
        self.optim = torch.optim.Adam(self.model.parameters(), self.learning_rate)

        # Throw an error if there are extra keyword arguments
        if len(kwargs) > 0:
            extra = ", ".join('"%s"' % k for k in list(kwargs.keys()))
            raise ValueError("Unrecognized arguments %s" % extra)

        self._reset()

        self.idx_to_word = idx_to_word

    def _reset(self):
        """
        Set up some book-keeping variables for optimization. Don't call this
        manually.
        """
        # Set up some variables for book-keeping
        self.epoch = 0
        self.loss_history = []

    ############################################################################
    # Req 2-5: single gradient update를 위한 _step() 함수 구현                     #
    ############################################################################
    def _step(self):
        """
        모델 학습 시 single gradient update를 수행합니다. train() 메소드 내에서 호출되며, 인위적으로 호출해서는 안됩니다.
        """
        # Make a minibatch of training data
        minibatch = sample_coco_minibatch(
            self.data, batch_size=self.batch_size, split="train"
        )
        captions, features, urls = minibatch
        captions_in = captions[:, :-1]
        captions_out = captions[:, 1:]

        mask = captions_out != self.model._null

        ############################################################################
        # TODO: single gradient update를 위한 _step() 함수 구현                        #
        # 학습을 위한 데이터 준비는 완료되었고, 해당 데이터로 single gradient update를 수행합니다.  #
        # 1) Minibatch 내의 데이터를 torch 텐서로 변환해 줍니다.                            #
        # 2) model에 image feature와 t_captions_in을 전달하여 예측한 값을 return 받습니다.   #
        # 3) Req 2-4에서 작성한 transformer_temporal_softmax_loss()를 호출하여 loss 값을  #
        #   측정합니다.                                                               #
        # 4) loss_history에 저장하고, backward() 스텝을 거친 후 optimizer를 업데이트 합니다.   #
        ############################################################################
        # *****START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****

        pass

        # *****END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****
        ############################################################################
        #                             END OF YOUR CODE                             #
        ############################################################################

    def train(self):
        """
        학습 시작을 위한 코드입니다.
        """
        num_train = self.data["train_captions"].shape[0]
        iterations_per_epoch = max(num_train // self.batch_size, 1)
        num_iterations = self.num_epochs * iterations_per_epoch

        for t in range(num_iterations):
            self._step()

            # Maybe print training loss
            if self.verbose and t % self.print_every == 0:
                print(
                    "(Iteration %d / %d) loss: %f"
                    % (t + 1, num_iterations, self.loss_history[-1])
                )

            # At the end of every epoch, increment the epoch counter.
            epoch_end = (t + 1) % iterations_per_epoch == 0

    ############################################################################
    # Req 2-4: 학습 loss 계산 코드 구현                                             #
    ############################################################################
    def transformer_temporal_softmax_loss(self, x, y, mask):
        """
        Timeseries 길이 T가 주어졌을 때, 각 timestep 별로 주어지는 size N minibatch 에서
        vocabulary size V에 대한 prediction을 수행합니다.
        입력 x는 모든 timestep의 token들에 해당하는 vocabulary를 예측하고, y는 각 timestep별로
        ground-truth vocabulary의 index를 의미합니다.
        결국 x에서 예측한 vocabulary와 y의 ground-truth 간의 각 timestep별 cross-entropy loss를
        측정하고, 이를 평균내어 최종 loss 값을 출력할 수 있습니다.

        중요한 점은, minibatch 내의 데이터 별로 예측한 문장의 길이가 다르기 때문에, 데이터 별로 특정
        timestep 까지의 loss만 측정하고, 나머지는 측정하길 원하지 않을 경우가 있습니다(NULL token으로 padding되어 있습니다).
        이때 mask 값을 활용하여 특정 timestep 이후로는 loss에 관여하지 않도록 loss 계산시 제거해줄 수 있습니다.

        Inputs:
        - x: Input scores, of shape (N, T, V)
        - y: Ground-truth indices, of shape (N, T) where each element is in the range
             0 <= y[i, t] < V
        - mask: Boolean array of shape (N, T) where mask[i, t] tells whether or not
          the scores at x[i, t] should contribute to the loss.
        Returns a tuple of:
        - loss: Scalar giving loss
        """

        ############################################################################
        # TODO: transformer 학습을 위한 loss 계산 코드를 구현                             #
        # 본 실습은 loss 값을 계산하기 위한 코드 구현입니다.                                  #
        # 예측한 문장과 ground-truth 문장 간의 cross-entropy loss를 측정합니다              #
        # 문장이 마친 이후에 해당하는 timestep은 mask 값을 활용하여 loss에 관여하지 않도록 합니다.   #
        ############################################################################
        # *****START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****

        pass

        # *****END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE)*****
        ############################################################################
        #                             END OF YOUR CODE                             #
        ############################################################################

        return loss