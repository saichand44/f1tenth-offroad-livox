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
    
    def unpack_and_save(self, num_robots, save_dir, filename):

        def to_numpy(x):
            if torch.is_tensor(x):
                return x.detach().cpu().numpy()

            if isinstance(x, np.ndarray):
                return x
            
            return x
    
        unpacked = {}
        keys = self.get_keys()

        # get the length of the data
        if 'timestamps' not in keys:
            raise KeyError("Missing 'timestamps' key; cannot determine data length.")
        data_length = len(getattr(self, 'timestamps'))

        unpacked['robot_id'] = []
        unpacked['timestamps'] = []
        unpacked['ground_plane_inclination'] = []
        unpacked['g_original'] = []
        unpacked['g_transform'] = []
        unpacked['g_R_p'] = []
        unpacked['target_velocity'] = []
        unpacked['target_steering'] = []
        unpacked['root_pose'] = []
        unpacked['root_velocity'] = []

        joint_names = getattr(self, 'joint_names')[0]
        for joint_name in joint_names:
            unpacked[f'joint_velocity_{joint_name}'] = []

        body_names = getattr(self, 'body_names')[0]
        for body_name in body_names:
            unpacked[f'root_acceleration_{body_name}'] = []

        for robot_id in range(num_robots):
            for data_idx in range(data_length):
                for key in keys:
                    # add the robot id
                    unpacked['robot_id'].append(robot_id)
                    unpacked['timestamps'].append(to_numpy(getattr(self, 'timestamps')[data_idx]))

                    unpacked['ground_plane_inclination'].append(to_numpy(getattr(self, 'ground_plane_inclination')))
                    unpacked['g_original'].append(to_numpy(getattr(self, 'g_original')))
                    unpacked['g_transform'].append(to_numpy(getattr(self, 'g_transform')))
                    unpacked['g_R_p'].append(to_numpy(getattr(self, 'g_R_p')))
                    unpacked['target_velocity'].append(to_numpy(getattr(self, 'target_velocity')[0][robot_id]))
                    unpacked['target_steering'].append(to_numpy(getattr(self, 'target_steering')[0][robot_id]))

                    if key == 'joint_velocity':
                        # append the joint velocity with joint names
                        joint_vel_all_data = getattr(self, key)[data_idx][robot_id]
                        for idx, joint_name in enumerate(joint_names):
                            unpacked[f'{key}_{joint_name}'].append(to_numpy(joint_vel_all_data[idx]))

                    elif key == 'root_acceleration':
                        # append the root acceleration with body names
                        root_acc_all_data = getattr(self, key)[data_idx][robot_id]
                        for idx, body_name in enumerate(body_names):
                            unpacked[f'{key}_{body_name}'].append(to_numpy(root_acc_all_data[idx]))

                    elif key in ['root_pose', 'root_velocity']:
                        data = getattr(self, key)[data_idx][robot_id]
                        unpacked[key].append(to_numpy(data))

                    else:
                        continue

        # print(f"unpacked keys: \n {unpacked.keys()}")
        # print(f"unpacked: \n {unpacked}")

        # save the data
        np.savez(os.path.join(save_dir, filename), **unpacked)

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