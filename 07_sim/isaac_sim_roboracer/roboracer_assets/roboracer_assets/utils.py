import numpy as np
import os
import torch

class Data:
    def __init__(self, *keys):
        self.init(*keys)

    def init(self, *keys):
        for key in keys:
            setattr(self, key, []) 
    
    def get_keys(self):
        return list(vars(self).keys())
    
    def list(self):
        print(self.get_keys())
            
    def pop(self, *keys, index=0):
        if len(keys) == 0:
            keys = self.get_keys()
        for key in keys:
            # print(key)
            getattr(self, key).pop(index)
    
    def unpack_and_save(self, num_robots):

        def to_numpy(x):
            return x.cpu().numpy() if torch.is_tensor(x) else x
        
        unpacked = {}
        keys = self.get_keys()

        for key in keys:
            if key not in ('joint_velocity', 'root_acceleration'):
                unpacked[key] = []

        for robot_id in range(num_robots):
            for key in keys:
                data = getattr(self, key)

                if key == 'joint_velocity':
                    pass
                elif key == 'root_acceleration':
                    pass
                else:
                    unpacked[key].append(to_numpy(data))

        print(unpacked)

    def save(self, *keys, save_dir=''):
        for key in keys:
            np.savez(save_dir + key, *getattr(self, key))
            
    def load(self, *keys, save_dir=''):
        for key in keys:
            setattr(self, key, list(np.load(save_dir + key + '.npz', allow_pickle=True).values()))
    
    def save_onefile(self, *keys, save_dir='', filename = 'data_record', compress=False):
        if len(keys) == 0:
            keys = self.get_keys()
        d = {}
        for key in keys:
            d[key] = {key: getattr(self, key)}
        if compress:
            np.savez_compressed(save_dir + filename, **d)
        else:
            np.savez(save_dir + filename, **d)
            
    def load_onefile(self, *keys, save_dir='', filename = 'data_record'):
        d = np.load(save_dir + filename + '.npz', allow_pickle=True)
        if len(keys) == 0:
            keys = list(d.keys())
        for key in keys:
            if hasattr(d[key][()][key], "__len__"):
                setattr(self, key, list(d[key][()][key]))
            else:
                setattr(self, key, d[key][()][key])