"""Explicit NON_PRODUCTION fixtures, NEVER loaded by default shadow launch."""
from dataclasses import replace
from pathlib import Path
import numpy as np
import yaml
from .pit_surface import PlanningSpatialContext
from .surface_mask import GrabMask


def fixture(geofence_template):
    gc=yaml.safe_load(Path(geofence_template).read_text(encoding='utf-8'))
    semantic=dict(config_state='VALID',schema_version=1,map_id='SYNTHETIC_PIT_MAP',semantic_version='synthetic-v1',frame_id='map',
        track=dict(track_id='synthetic-track',origin_map_m=[0,0],direction_map_xy=[1,0],valid_x_range_m=[-3,3]),
        pits=[dict(pit_id='SYNTHETIC_PIT',polygon_map_m=[[-2,-2],[2,-2],[2,2],[-2,2]],
                   pit_bottom_z_m=0.,wall_top_z_m=3.,safety_inset_m=.1)],
        unloading_stations=[dict(station_id='synthetic-only',target_pose_map=[0,0,0],roi_polygon_map_m=[[-.1,-.1],[.1,-.1],[0,.1]])],
        safe_wait_pose=[0,0,0],restricted_regions=[],calibration_anchors=[])
    gc.update(config_state='VALID',config_version='SYNTHETIC_NON_PRODUCTION',geofence_id='synthetic-geofence',
        geofence_version='synthetic-v1',semantic_map_id=semantic['map_id'],semantic_version=semantic['semantic_version'],
        pit_id='SYNTHETIC_PIT',source_type='SYNTHETIC',evidence_timeout_sec=.7,direction_probe_m=.1)
    gc['x']['calibration_id']='synthetic-x';gc['y']['calibration_id']='synthetic-y'
    gc['y'].update(negative_limit_signal='y_left_limit',positive_limit_signal='y_right_limit',limit_semantics='PIT_DIRECTION_LIMIT',
        negative_expected_window_m=[-2.3,-2.1],positive_expected_window_m=[2.1,2.3])
    gc['axis_to_map'].update(state='SYNTHETIC_VALIDATED',calibration_id='synthetic-axis-map',map_id=semantic['map_id'],
        semantic_version=semantic['semantic_version'],pit_id='SYNTHETIC_PIT',x_calibration_id='synthetic-x',y_calibration_id='synthetic-y',
        origin_map_xy_m=[0,0],x_direction_map_xy=[1,0],y_direction_map_xy=[0,1],x_scale=1,y_scale=1,x_offset_m=0,y_offset_m=0)
    gc['static_margin'].update(grab_horizontal_radius_m=.3,maximum_swing_m=.02,position_uncertainty_m=.02,
        calibration_uncertainty_m=.02,stop_margin_m=.02,additional_margin_m=.02)
    c=dict(contract_version=1,config_state='VALID',config_version='SYNTHETIC_NON_PRODUCTION',source_type='SYNTHETIC',time_domain='ROS_TIME',
        pit_id='SYNTHETIC_PIT',resolution_m=.25,frame_ttl_sec=.7,surface_ttl_sec=1.2,window_sec=1.,maximum_frame_span_sec=.03,
        epoch_tolerance_sec=1e-6,sensor_uncertainty_m=.01,single_sensor_extra_uncertainty_m=.02,mad_scale=3.,max_height_dispersion_m=.3,
        minimum_neighborhood_coverage=.5,minimum_cell_samples=4,minimum_cell_frames=2,maximum_input_points=20000,
        maximum_grid_cells=1024,maximum_window_frames=4,maximum_candidates=128,dynamic_mask_dilation_m=.05,
        static_masks_state='VALID',static_masks=[],kernel=dict(state='SYNTHETIC_NON_PRODUCTION',kernel_id='synthetic-kernel',
            provenance='NON_PRODUCTION_RECTANGLE',footprint_xy_m=[[-.25,-.25],[.25,-.25],[.25,.25],[-.25,.25]],
            capture_depth_m=.3,yaw_control_available=False,yaw_mode='FIXED_MECHANICAL',fixed_yaw_rad=0.),
        planning=dict(minimum_observed_fraction=1.,maximum_unknown_fraction=0.,maximum_slope_rad=.6,maximum_roughness_m=.1,
            wall_margin_m=.1,distance_acceleration_enabled=True,recent_grab_radius_m=.75,recent_grab_ttl_sec=3.,
            disturbance_dilation_m=.05,maximum_recent_events=4),
        physical_output_enabled=False,automatic_control_enabled=False,camera_control_authority=False)
    ctx=PlanningSpatialContext('synthetic_cloud','map',semantic['map_id'],semantic['semantic_version'],'SYNTHETIC_PIT',
        'synthetic-map-transform','synthetic-extrinsic','SYNTHETIC_VALIDATED','SYNTHETIC',10.,.01,tuple(np.eye(4).ravel()),
        ('synthetic-er1-0','synthetic-er1-1'),True,'synthetic-stationary-window')
    grab=GrabMask(10.,'map',semantic['map_id'],semantic['semantic_version'],'SYNTHETIC_PIT','synthetic-extrinsic','SYNTHETIC',
        'TRACKED',True,True,((-.4,-.4),(.4,-.4),(.4,.4),(-.4,.4)),.85,1.8,0.,'SYNTHETIC_VALIDATED')
    return c,semantic,gc,ctx,grab


SCENARIOS=('flat','single_mound','multiple_mounds','ridge','valley','sloped','rough','holes','partial',
           'wall_high','concave','grab_occlusion','grab_connected','sensor0','sensor1','dropout','stale_transform',
           'map_mismatch','stale_surface','disturbance','reobservation')


def scene(grid,name,rng=None):
    rng=np.random.default_rng(20261013) if rng is None else rng
    xy=np.array([grid.center(i) for i in range(grid.width*grid.height)])
    x,y=xy.T
    z=np.ones(len(x))
    if name=='single_mound': z+=np.exp(-2*(x*x+y*y))*.6
    if name=='multiple_mounds': z+=.6*np.exp(-3*((x-.8)**2+y*y))+.4*np.exp(-3*((x+.8)**2+(y-.5)**2))
    if name=='ridge': z+=.4*np.exp(-4*y*y)
    if name=='valley': z-=.4*np.exp(-4*y*y)
    if name=='sloped': z+=.2*x
    if name=='rough': z+=rng.uniform(-.3,.3,len(z))
    if name=='wall_high': z+=.8*(x>1.3)
    p=np.repeat(np.c_[xy,z],6,axis=0)
    sensors=np.tile([0,1,0,1,0,1],len(xy))
    if name=='holes':
        keep=np.linalg.norm(p[:,:2],axis=1)>.65;p,sensors=p[keep],sensors[keep]
    if name=='partial':
        keep=p[:,0]>0;p,sensors=p[keep],sensors[keep]
    if name=='sensor0': sensors[:]=0
    if name=='sensor1': sensors[:]=1
    if name in ('grab_occlusion','grab_connected'):
        inside=(np.abs(p[:,0])<.4)&(np.abs(p[:,1])<.4)
        p[inside,2]=1.1
    return p,sensors


def ingest(builder,p,sensors,context,grab,epoch,counter):
    return builder.ingest(p,sensors,dict(epoch=epoch,source_counter=counter,frame_id=context.source_frame,span_sec=.02,
        source_type=context.source_type,calibration_id=context.calibration_id),replace(context,measurement_epoch=epoch),
        replace(grab,epoch=epoch),epoch+.01)
