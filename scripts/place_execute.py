import argparse
import os
import sys

import hydra
import numpy as np
import open3d as o3d
from omegaconf import DictConfig, OmegaConf

from rtc_core.place_skill.place_execute import ExecutePlace

def test_taxpose_wp(place_execute: ExecutePlace):
    ## Load data
    # pcd_dir = "/home/mfi/repos/rtc_vision_toolbox/data/demonstrations/06-20-wp/teach_data"
    # gripper_close_up_pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, "pcd_data/demo3_gripper_close_up_cam2_closeup_pointcloud.ply"))
    # ih_camera_view_pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, "pcd_data/demo3_ih_camera_view_cam3_gripper_pointcloud.ply"))
    # ih_camera_view_pose = np.load(os.path.join(pcd_dir, "pose_data/demo1_ih_camera_view_pose.npy"))

    # pcd_dir = "/home/mfi/repos/rtc_vision_toolbox/data/demonstrations/07-24-wp/execute_data/0728_1225"
    pcd_dir = "/home/mfi/repos/rtc_vision_toolbox/data/demonstrations/07-24-wp/teach_data"
    gripper_close_up_pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, "pcd_data/demo5_gripper_close_up_cam2_closeup_pointcloud.ply"))

    pcd_dir = "/home/mfi/repos/rtc_vision_toolbox/data/demonstrations/07-24-wp/teach_data"    
    # pcd_dir = "/home/mfi/repos/rtc_vision_toolbox/data/demonstrations/07-24-wp/execute_data/0728_1225"
    ih_camera_view_pcd = o3d.io.read_point_cloud(os.path.join(pcd_dir, "pcd_data/demo5_ih_camera_view_cam3_gripper_pointcloud.ply"))
    ih_camera_view_pose = np.load(os.path.join(pcd_dir, "pose_data/demo5_ih_camera_view_pose.npy"))
    
    place_execute.predict_placement_pose_data['action']['pcd'] = gripper_close_up_pcd
    place_execute.predict_placement_pose_data['anchor']['pcd'] = ih_camera_view_pcd
    place_execute.predict_placement_pose_data['anchor']['eef_pose'] = ih_camera_view_pose
    
    predicted_pose = place_execute.infer_placement_pose()
    
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, help="Path to the configuration file.")
    args, unknown = parser.parse_known_args()
    config_file = args.config

    # Filter out custom arguments before Hydra processes them
    filtered_argv = [arg for arg in sys.argv if arg.startswith("--hydra")]

    # Manually call Hydra main
    sys.argv = [sys.argv[0]] + filtered_argv        
    
    # Read configuration file

    config_dir = os.path.dirname(config_file)
    config_name = os.path.basename(config_file)
    
    print(f"Reading configuration file: {config_file}")
    print(f"Configuration directory: {config_dir}")
    print(f"Configuration name: {config_name}")
    
    hydra.initialize(config_path=config_dir, version_base="1.3")
    config: DictConfig = hydra.compose(config_name)
    
    print(OmegaConf.to_yaml(config, resolve=True))

    place_execute = ExecutePlace(config)

    # pose = place_execute.devices.robot_get_eef_pose()
    # np.save('start_pose.npy', pose)
    # print(pose)
    # breakpoint()
    
    # place_execute.collect_data('ih_camera_view')
    # place_execute.collect_data('check_cal')

    # pose = np.load('/home/mfi/repos/rtc_vision_toolbox/data/demonstrations/07-24-wp/execute_data/0728_1225/predicted_pose.npy')
    # new_pose = pose + place_execute.cfg.execution.target.bias
    # breakpoint()
    # pose = np.dot(new_pose, np.eye(4))
    # place_execute.devices.robot_move_to_pose(pose)
    
    # test_taxpose_wp(place_execute) 
    place_execute.execute()
    