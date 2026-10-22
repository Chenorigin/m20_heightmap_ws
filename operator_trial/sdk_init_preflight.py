#!/usr/bin/env python3
"""Bounded read-only live readiness gate; never constructs a motor publisher."""
import argparse
import os
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu
from nav_msgs.msg import Odometry
from drdds.msg import JointsDataCmd
from std_msgs.msg import String
from m20_heightmap_msgs.msg import TerrainObservation
from m20_heightmap.runtime import load_config
from m20_heightmap.receiver import receive_terrain
from m20_heightmap.commission import geometry_hash,commission_matches
from m20_heightmap.cloud import read_cloud,validate_cloud_time
from m20_heightmap.ownership import check_command_endpoints
from m20_heightmap.native_packet import validate_receipt

from terrain_init_guard import validate_envelope, WAIVED_INIT_QUALITY_ERRORS
ROOT=Path(os.environ['M20_TRIAL_RELEASE']).resolve()
if ROOT.name != '20261015_1d75c14e4ab5':
    raise ValueError('Unexpected immutable trial release')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--profile',choices=['v0','v1'],default='v1');ap.add_argument('--control',action='store_true');ap.add_argument('--seconds',type=float,default=8);args=ap.parse_args()
    if not 3<=args.seconds<=20:raise ValueError('read-only probe is bounded to 3..20 seconds')
    config_path=ROOT/'runtime'/('heightmap_'+args.profile+'.local.json')
    cfg=load_config(config_path)
    if not args.control:raise ValueError('Initialization-only probe requires --control and real commission receipt')
    if args.control:
        commission=json.loads((ROOT/'runtime/commissioned.json').read_text())
        for key in ('factory_geometry_checked','software_limits_checked','operator_controls_start','physical_estop_available'):
            if commission.get(key) is not True:raise ValueError('commissioning check missing: '+key)
        if commission.get('profile')!=args.profile:raise ValueError('commissioning profile mismatch')
        if not commission_matches(ROOT,config_path,args.profile):raise ValueError('commissioning configuration or boot changed; repeat physical checks')
    expected_geometry=geometry_hash(ROOT,config_path)
    rclpy.init();node=rclpy.create_node('m20_student_readonly_preflight')
    records={'cloud':[],'imu':[],'pose':[],'terrain':[],'motor_command':[]};errors=[]
    terrain_watermark={};waived=[];last_terrain_transport_stamp=None
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    cloud_q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
    aos=Path('/opt/robot/os_type').read_text().strip()=='CA9B_AOS'
    cloud_identity={'session':None,'sequence':0}
    def cloud_receipt(msg):
        try:
            value=json.loads(msg.data);when=time.monotonic()
            stamp,_=validate_receipt(value,cfg,node.get_clock().now().nanoseconds*1e-9)
            if cloud_identity['session'] is not None and value['session']!=cloud_identity['session']:raise ValueError('GOS cloud session changed')
            if value['sequence']<=cloud_identity['sequence']:raise ValueError('GOS cloud sequence repeated/regressed')
            cloud_identity.update(session=value['session'],sequence=value['sequence'])
            records['cloud'].append((when,stamp))
        except (ValueError,KeyError,TypeError) as exc:errors.append(str(exc))
    def callback(kind,msg):
        nonlocal last_terrain_transport_stamp
        when=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if kind=='motor_command':records[kind].append((when,stamp));return
        age=node.get_clock().now().nanoseconds*1e-9-stamp
        try:
            if kind=='cloud':
                r=cfg['runtime']
                _,offsets=read_cloud(msg,r['max_cloud_points'],r['max_cloud_bytes'],r['point_time_field'],r['point_time_scale_s'],r['point_time_mode'])
                _,end=validate_cloud_time(stamp,node.get_clock().now().nanoseconds*1e-9,r,offsets)
                age=node.get_clock().now().nanoseconds*1e-9-end
            if not np.isfinite(age) or age< -cfg['runtime']['future_tolerance_s'] or age>cfg['runtime']['input_timeout_s']:raise ValueError(kind+' stamp stale/future')
            if kind=='cloud' and msg.header.frame_id not in cfg['lidar_extrinsics']:raise ValueError('cloud frame')
            if kind=='pose' and (msg.header.frame_id!=cfg['frames']['world'] or msg.child_frame_id!=cfg['frames']['base']):raise ValueError('base pose frame')
            if kind=='terrain':
                validated_now=node.get_clock().now().nanoseconds*1e-9
                validate_envelope(msg,validated_now,cfg,expected_geometry,terrain_watermark)
                last_terrain_transport_stamp=stamp
                try:
                    receive_terrain(msg,validated_now,cfg)
                except ValueError as exc:
                    if str(exc) not in WAIVED_INIT_QUALITY_ERRORS:raise
                    waived.append(str(exc))
                if not msg.motion_compensated:raise ValueError('terrain is not odometry-compensated')
                if args.control and (not msg.operator_commissioned or msg.geometry_hash!=expected_geometry):raise ValueError('GOS geometry or commissioning differs from AOS')
            records[kind].append((when,stamp))
        except ValueError as exc:errors.append(str(exc))
    subs=[]
    for kind,typ,topic in [('cloud',PointCloud2,cfg['topics']['cloud']),('imu',Imu,cfg['topics']['imu']),('pose',Odometry,cfg['topics']['odom']),('terrain',TerrainObservation,cfg['topics']['terrain'])]:
        if kind=='cloud' and aos:
            subs.append(node.create_subscription(String,'/m20/gos/cloud_receipt',cloud_receipt,cloud_q))
            continue
        subs.append(node.create_subscription(typ,topic,lambda m,k=kind:callback(k,m),cloud_q if kind=='cloud' else q))
    if args.control:subs.append(node.create_subscription(JointsDataCmd,'/JOINTS_CMD',lambda m:callback('motor_command',m),q))
    start=time.monotonic()
    while time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.02)
    now=time.monotonic();checks={};passed=True
    for kind in ('cloud','imu','pose','terrain'):
        values=records[kind];unique=len(set(v[1] for v in values));tail=now-values[-1][0] if values else None
        required=3 if kind=='cloud' else 10
        max_gap=max(np.diff([v[0] for v in values])) if len(values)>1 else None
        ok=bool(unique>=required and tail is not None and tail<cfg['runtime']['input_timeout_s'] and max_gap is not None and max_gap<cfg['runtime']['input_timeout_s'])
        checks[kind]={'samples':len(values),'unique_stamps':unique,'last_receive_age_s':tail,'max_receive_gap_s':max_gap,'passed':ok};passed &= ok
    for topic in (cfg['topics']['cloud'],cfg['topics']['terrain']):
        count=len(node.get_publishers_info_by_topic(topic));checks[topic]={'publishers':count,'passed':count==1};passed &= count==1
    if aos:
        count=len(node.get_publishers_info_by_topic('/m20/gos/cloud_receipt'))
        checks['/m20/gos/cloud_receipt']={'publishers':count,'passed':count==1};passed &= count==1
    if args.control:
        commands=node.get_publishers_info_by_topic('/JOINTS_CMD')
        try:
            vendor_pid=int(os.environ['M20_VERIFIED_VENDOR_PID'])
            identity=check_command_endpoints(node.get_publishers_info_by_topic('/JOINTS_DATA'),commands,vendor_pid=vendor_pid)
            quiet=not records['motor_command']
        except ValueError as exc:
            identity={'error':str(exc)};quiet=False
        checks['existing_motor_stream']={'messages':len(records['motor_command']),'publishers':len(commands),'identity':identity,'passed':quiet};passed &= quiet
    if (last_terrain_transport_stamp is None or
            not -cfg['runtime']['future_tolerance_s'] <= node.get_clock().now().nanoseconds*1e-9-last_terrain_transport_stamp <= cfg['runtime']['input_timeout_s']):
        errors.append('Last terrain transport no longer fresh at initialization decision')
    passed &= not errors
    result={'sdk_initialization_ready':bool(passed),'scope':'USER_AUTHORIZED_INITIALIZATION_ONLY; terrain quality not asserted; original runtime validation unchanged','policy_readiness_not_asserted':True,'waived_terrain_quality_errors':{e:waived.count(e) for e in set(waived)},'profile':args.profile,'checks':{('terrain_transport_for_initialization' if k=='terrain' else k):v for k,v in checks.items()},'errors':sorted(set(errors)),'cloud_evidence':'GOS per-frame verified native DDS source receipt' if aos else 'local PointCloud2','motor_publishers_created':0}
    print(json.dumps(result,indent=2));node.destroy_node();rclpy.shutdown()
    if not passed:raise SystemExit(1)

if __name__=='__main__':main()
