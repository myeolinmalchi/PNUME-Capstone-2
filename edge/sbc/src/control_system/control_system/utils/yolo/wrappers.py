from abc import abstractmethod, ABC
from typing import Any, Generic, TypeVar, Union, Tuple, List
from rknnlite.api import RKNNLite


I = TypeVar("I")
O = TypeVar("O")

class ModelWrapper(ABC):
    @abstractmethod
    def __init__(self, model_path: str, target = None, device_id = None):  ...

    @abstractmethod
    def run(self, inputs): ...

    @abstractmethod
    def release(self): ...

    @staticmethod
    def setup(model_path: str):
        if model_path.endswith('.rknn'):
            platform = 'rknn'
            model = RKNNWrapper(model_path)

        else:
            assert False, "{} is not rknn/pytorch/onnx model".format(model_path)
        print('Model-{} is {} model, starting val'.format(model_path, platform))
        return model, platform



class RKNNWrapper(ModelWrapper):

    def __init__(self, model_path) -> None:
        rknn = RKNNLite()

        # Direct Load RKNN Model
        rknn.load_rknn(model_path)

        print('--> Init runtime environment')
        ret = rknn.init_runtime()
        if ret != 0:
            print('Init runtime environment failed')
            exit(ret)

        print('done')
        
        self.rknn = rknn

    def run(self, inputs):
        if self.rknn is None:
            print("ERROR: rknn has been released")
            return []

        if isinstance(inputs, list) or isinstance(inputs, tuple):
            pass
        else:
            inputs = [inputs]

        result = self.rknn.inference(inputs=inputs)
        return result

    def release(self):
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None