"TBW baseline fixed wing and span location geometry working? backup"

# import numpy as np
import numpy as np
import caddee.api as cd 
import lsdo_geo as lg
import m3l
from caddee import GEOMETRY_FILES_FOLDER
from caddee.utils.helper_functions.geometry_helpers import make_rotor_mesh, make_vlm_camber_mesh, make_1d_box_beam_mesh, compute_component_surface_area, BladeParameters
from caddee.utils.aircraft_models.drag_models.drag_build_up import DragComponent
import lsdo_geo.splines.b_splines as bsp
import gc
gc.enable()
from dataclasses import dataclass, field
import csdl
import math

# @dataclass
# class geometryOutputs: 
#     wing_AR : m3l.Variable
#     # S_ref : m3l.Variable 
#     wing_area : m3l.Variable
#     # wing_span : m3l.Variable

# region True or False
geometry_plot = False
components_plot = False
do_plots = False
print_coordinates = False
plot_htail_mesh = False
plot_left_strut_mesh = False
plot_right_strut_mesh = False

# region Plots 
# VLM 
mesh_flag_wing = False
mesh_flag_wing_oml = False
mesh_flag_htail = False
mesh_flag_htail_oml = False
mesh_flag_left_strut = False
mesh_flag_left_strut_oml = False
mesh_flag_right_strut = False
mesh_flag_right_strut_oml = False
mesh_flag_vtail_helper = False
mesh_flag_fuselage = False
mesh_flag_left_jury_helper = False
mesh_flag_right_jury_helper = False
mesh_flag_gearpod = False
plot_unupdated_vlm_mesh = False
plot_updated_vlm_mesh = False
# beam
wing_beam_mesh_flag = False
left_strut_beam_mesh_flag = False
right_strut_beam_mesh_flag = False
left_jury_beam_mesh_flag = False
right_jury_beam_mesh_flag = False
# endregion

# endregion

# Instantiate system model
system_model = m3l.Model()

# Importing and refitting the geometrywing_oml_mesh
geometry = lg.import_geometry(GEOMETRY_FILES_FOLDER / 'tbw_try.stp', parallelize=True)
geometry.refit(parallelize=True, order=(4, 4))

if geometry_plot:
    geometry.plot()

plot_meshes = False
# geometry_dv = True

# region Declaring Components
wing = geometry.declare_component(component_name='wing', b_spline_search_names=['Wing'])
if components_plot:
    wing.plot()
htail = geometry.declare_component(component_name='htail', b_spline_search_names=['Htail'])
if components_plot:
    htail.plot()
strut = geometry.declare_component(component_name='strut', b_spline_search_names=['Strut'])
if components_plot:
    strut.plot()
strut_right = geometry.declare_component(component_name='strut_right', b_spline_search_names=['Strut, 0'])
if components_plot:
    strut_right.plot()
strut_left = geometry.declare_component(component_name='strut_left', b_spline_search_names=['Strut, 1'])
if components_plot:
    strut_left.plot()
jury = geometry.declare_component(component_name='jury', b_spline_search_names=['Jury'])
if components_plot:
    jury.plot()
jury_right = geometry.declare_component(component_name='jury_right', b_spline_search_names=['Jury, 0'])
if components_plot:
    jury_right.plot()
jury_left = geometry.declare_component(component_name='jury_left', b_spline_search_names=['Jury, 1'])
if components_plot:
    jury_left.plot()
vtail = geometry.declare_component(component_name='vtail', b_spline_search_names=['Vtail'])
if components_plot:
    vtail.plot()
Fuselage = geometry.declare_component(component_name='Fuselage', b_spline_search_names=['Fuselage'])
if components_plot:
    Fuselage.plot()
gearpod = geometry.declare_component(component_name='gearpod', b_spline_search_names=['Gear Pod'])
if components_plot:
    gearpod.plot()
# endregion

# region geometry dv

# geometry_dv = False
geometry_dv = True

# wing_span_dv = system_model.create_input(name='wing_span_dv', shape = (1,), val = 170, dv_flag = geometry_dv, lower = 140., upper = 200., scaler= 1.e-1)
wing_span_dv = system_model.create_input(name='wing_span_dv', shape = (1,), val = 171, dv_flag = geometry_dv, lower = 140., upper = 200.)

geometry_dv = False
# geometry_dv = True
# wing_tip_chord_left_dv = system_model.create_input(name='wing_tip_chord_left_dv', shape= (1,), val = 10.7, dv_flag = geometry_dv, lower=2.37, upper=5.13)
wing_tip_chord_left_dv = system_model.create_input(name='wing_tip_chord_left_dv', shape= (1,), val = 3.8, dv_flag = geometry_dv, lower=3.37, upper=4.13)
# wing_tip_chord_right_dv = system_model.create_input(name='wing_tip_chord_right_dv', shape= (1,), val = 3.8, dv_flag = geometry_dv, lower=3.37, upper=4.13, scaler =1.e-1)
wing_root_chord_dv = system_model.create_input(name='wing_root_chord_dv', shape= (1,), val = 10.7, dv_flag = geometry_dv, lower=9.64, upper=11.79)
# wing_root_chord_dv = system_model.create_input(name='wing_root_chord_dv', shape= (1,), val = 9.64, dv_flag = geometry_dv, lower=9.64, upper=12.79)
# wing_mid_chord_left_dv = system_model.create_input(name = 'wing_mid_chord_left_dv', shape=(1,), val= 11.68, dv_flag = geometry_dv, lower = 7.70, upper = 11.68)
wing_mid_chord_left_dv = system_model.create_input(name = 'wing_mid_chord_left_dv', shape=(1,), val= 9.5, dv_flag = geometry_dv, lower = 8.70, upper = 10.68)
# wing_mid_chord_left_dv = system_model.create_input(name = 'wing_mid_chord_left_dv', shape=(1,), val= 8.7, dv_flag = geometry_dv, lower = 8.70, upper = 10.68)
# wing_mid_chord_right_dv = system_model.create_input(name = 'wing_mid_chord_right_dv', shape=(1,), val= 9.5, dv_flag = geometry_dv, lower = 8.70, upper = 10.68, scaler =1.e-1)

# left_strut_chord_dv = system_model.create_input(name='left_strut_chord_dv', shape = (1,), val = 1.8, dv_flag = geometry_dv, lower = 1.7, upper = 1.9)
# right_strut_chord_dv = system_model.create_input(name='right_strut_chord_dv', shape = (1,), val = 1.8, dv_flag = geometry_dv, lower = 1.7, upper = 1.9)
# left_jury_chord_dv = system_model.create_input(name='left_strut_chord_dv', shape = (1,), val = 1.8, dv_flag = geometry_dv, lower = 1.7, upper = 1.9)
# right_jury_chord_dv = system_model.create_input(name='right_strut_chord_dv', shape = (1,), val = 1.8, dv_flag = geometry_dv, lower = 1.7, upper = 1.9)

# endregion

# region mesh

# region Wing Mesh

num_spanwise_wing_vlm = 25
num_chordwise_wing_vlm = 7 

point00_wing = np.array([68.035, 85.291, 4.704]) # * ft2m # Right tip leading edge 
wing_le_right = point00_wing
point01_wing = np.array([71.790, 85.291, 4.704]) # * ft2m # Right tip trailing edge
wing_te_right = point01_wing
point10_wing = np.array([57.575, 49.280, 5.647]) # * ft2m # Right leading edge 1
point11_wing = np.array([67.088, 49.280, 5.507]) # * ft2m # Right trailing edge 1
point20_wing = np.array([56.172, 42.646, 5.821]) # * ft2m # Right leading edge 2
point21_wing = np.array([65.891, 42.646, 5.651]) # * ft2m # Right trailing edge 2
point30_wing = np.array([47.231, 0.000, 6.937]) # * ft2m # Center Leading Edge
wing_le_center = point30_wing
point31_wing = np.array([57.953, 0.000, 6.566]) # * ft2m # Center Trailing edge
wing_te_center = point31_wing
point40_wing = np.array([56.172, -42.646, 5.821]) # * ft2m # Left leading edge 2
point41_wing = np.array([65.891, -42.646, 5.651]) # * ft2m # Left trailing edge 2
point50_wing = np.array([57.575, -49.280, 5.647]) # * ft2m # Left leading edge 1
point51_wing = np.array([67.088, -49.280, 5.507]) # * ft2m # Left trailing edge 1
point60_wing = np.array([68.035, -85.291, 4.704]) # * ft2m # Left tip leading edge 
wing_le_left = point60_wing
point61_wing = np.array([71.790, -85.291, 4.704]) # * ft2m # Left tip trailing edge
wing_te_left = point61_wing

wing_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=wing,
    num_spanwise=num_spanwise_wing_vlm,
    num_chordwise=num_chordwise_wing_vlm,
    te_right=np.array([73., 85., 5.507]),
    te_left=np.array([73., -85., 5.507]),
    te_center=np.array([57.953, 0.000, 6.566]),
    le_left=np.array([66., -85., 5.647]),
    le_right=np.array([66., 85., 5.647]),
    le_center=np.array([47.231, 0.000, 6.937]),
    grid_search_density_parameter=100,
    le_interp='linear',
    te_interp='linear',
    off_set_x=0.2,
    bunching_cos=True,
    plot=mesh_flag_wing,
    mirror=True,
)

# endregion

# region htail mesh

num_spanwise_htail_vlm = 21
num_chordwise_htail_vlm = 5

num_spanwise_vlm_htail = 9
num_chordwise_vlm_htail = 4

point00_htail = np.array([132.002-10.0, 19.217+4.5, 18.993+3.5]) # * ft2m # Right tip leading edge
point01_htail = np.array([135.993, 19.217, 18.993]) # * ft2m # Right tip trailing edge
point10_htail = np.array([122.905, 0.000, 20.000]) # * ft2m # Center Leading Edge
point11_htail = np.array([134.308, 0.000, 20.000]) # * ft2m # Center Trailing edge
point20_htail = np.array([132.002-10, -19.217-4.5, 18.993+3.5]) # * ft2m # Left tip leading edge
point21_htail = np.array([135.993, -19.217, 18.993]) # * ft2m # Left tip trailing edge

htail_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=htail,
    # num_spanwise=num_spanwise_htail_vlm,
    num_spanwise=num_spanwise_vlm_htail,
    # num_chordwise=num_chordwise_htail_vlm,
    num_chordwise=num_chordwise_vlm_htail,
    te_right=point01_htail,
    te_left=point21_htail,
    te_center=point11_htail,
    le_left=point20_htail,
    le_right=point00_htail,
    le_center=point10_htail,
    grid_search_density_parameter=100,
    plot=mesh_flag_htail,
    mirror=True,
)

# endregion

# region strut mesh

# region left strut mesh 

num_spanwise_strut_vlm = 21
num_chordwise_strut_vlm = 5 

vertex00_left_strut = np.array([55.573, -12.641, -4.200]) # left leading 1
vertex01_left_strut = np.array([55.034, -16.277, -3.204]) # left leading 2
vertex02_left_strut = np.array([56.422, -25.365, -0.713]) # left leading 3
vertex03_left_strut = np.array([58.313, -30.818, 0.782]) # left leading 4
vertex04_left_strut = np.array([58.089, -36.271, 2.276]) # left leading 5
vertex05_left_strut = np.array([59.477, -45.359, 4.767]) # left leading 6
vertex06_left_strut = np.array([61.090, -48.994, 5.763]) # left leading 7

vertex10_left_strut = np.array([57.309, -12.641, -4.200]) # left trailing 1
vertex11_left_strut = np.array([58.959, -16.277, -3.204]) # left trailing 2
vertex12_left_strut = np.array([60.348, -25.365, -0.713]) # left trailing 3
vertex13_left_strut = np.array([60.124, -30.818, 0.782]) # left trailing 4
vertex14_left_strut = np.array([62.014, -36.271, 2.276]) # left trailing 5
vertex15_left_strut = np.array([63.403, -45.359, 4.767]) # left trailing 6                
vertex16_left_strut = np.array([62.902, -48.994, 5.763]) # left trailing 7

left_strut_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=strut_left,
    num_spanwise=num_spanwise_strut_vlm,
    num_chordwise=num_chordwise_strut_vlm,
    te_right=vertex10_left_strut,
    # te_right=np.array([57.,-13.,-4.2]),
    te_left=vertex16_left_strut,
    # te_left=np.array([63., -49., 5.763]),
    # te_center=vertex13_left_strut,
    # te_center=np.array([60., -30.9, 0.782]),
    le_left=vertex06_left_strut,
    # le_left=np.array([61., -49., 5.763]),
    le_right=vertex00_left_strut,
    # le_right=np.array([55., -13., -4.200]),
    # le_center=vertex03_left_strut,
    # le_center=np.array([58., -30.8, 0.782]),
    grid_search_density_parameter=100,
    plot=mesh_flag_left_strut,
    mirror=False,
)

# endregion
  
# region right strut mesh

num_spanwise_strut_vlm = 21
num_chordwise_strut_vlm = 5 
vertex00_right_strut = np.array([55.573, 12.641, -4.200]) # right leading 1
vertex01_right_strut = np.array([55.034, 16.277, -3.204]) # right leading 2
vertex02_right_strut = np.array([56.422, 25.365, -0.713]) # right leading 3
vertex03_right_strut = np.array([58.313, 30.818, 0.782]) # right leading 4
vertex04_right_strut = np.array([58.089, 36.271, 2.276]) # right leading 5
vertex05_right_strut = np.array([59.477, 45.359, 4.767]) # right leading 6
vertex06_right_strut = np.array([61.090, 48.994, 5.763]) # right leading 7

vertex10_right_strut = np.array([57.309, 12.641, -4.200]) # right trailing 1
vertex11_right_strut = np.array([58.959, 16.277, -3.204]) # right trailing 2
vertex12_right_strut = np.array([60.348, 25.365, -0.713]) # right trailing 3
vertex13_right_strut = np.array([60.124, 30.818, 0.782]) # right trailing 4
vertex14_right_strut = np.array([62.014, 36.271, 2.276]) # right trailing 5
vertex15_right_strut = np.array([63.403, 45.359, 4.767]) # right trailing 6                
vertex16_right_strut = np.array([62.902, 48.994, 5.763]) # right trailing 7

right_strut_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=strut_right,
    num_spanwise=num_spanwise_strut_vlm,
    num_chordwise=num_chordwise_strut_vlm,
    te_right=vertex10_right_strut,
    # te_right=np.array([57.,-13.,-4.2]),
    te_left=vertex16_right_strut,
    # te_left=np.array([63., -49., 5.763]),
    # te_center=vertex13_right_strut,
    # te_center=np.array([60., -30.9, 0.782]),
    le_left=vertex06_right_strut,
    # le_left=np.array([61., -49., 5.763]),
    le_right=vertex00_right_strut,
    # le_right=np.array([55., -13., -4.200]),
    # le_center=vertex03_right_strut,
    # le_center=np.array([58., -30.8, 0.782]),
    grid_search_density_parameter=100,
    plot=mesh_flag_right_strut,
    mirror=False,
)

# endregion

# endregion

# region vtail mesh 
num_spanwise_vlm_vtail = 8
num_chordwise_vlm_vtail = 6
vtail_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=vtail,
    num_spanwise=num_spanwise_vlm_vtail,
    num_chordwise=num_chordwise_vlm_vtail,
    le_left=np.array([105.343, 0., 3.5,]), 
    le_right=np.array([120.341, 0., 20.754,]),
    te_left=np.array([122.596, 0., 3.5,]),
    te_right=np.array([137.595, 0., 20.754,]),
    plot=mesh_flag_vtail_helper,
    orientation='vertical',
    zero_y=True,
)
# endregion

# region jury mesh 

# region left jury mesh     
num_spanwise_vlm_jury = 5
num_chordwise_vlm_jury = 3
left_jury_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=jury_left,
    num_spanwise=num_spanwise_vlm_jury,
    num_chordwise=num_chordwise_vlm_jury,
    le_left=np.array([58.721, -30.818, 0.732,]), 
    le_right=np.array([58.052, -30.818, 6.181]),
    te_left=np.array([59.676, -30.818, 0.732]),
    te_right=np.array([59.237, -30.818, 6.181]),
    plot=mesh_flag_left_jury_helper,
    orientation='vertical',
    zero_y=True,
)
# endregion

# region right jury mesh     
num_spanwise_vlm_jury = 5
num_chordwise_vlm_jury = 3
right_jury_meshes = make_vlm_camber_mesh(
    geometry=geometry,
    wing_component=jury_right,
    num_spanwise=num_spanwise_vlm_jury,
    num_chordwise=num_chordwise_vlm_jury,
    le_left=np.array([58.721, 30.818, 0.732,]), 
    le_right=np.array([58.052, 30.818, 6.181]),
    te_left=np.array([59.676, 30.818, 0.732]),
    te_right=np.array([59.237, 30.818, 6.181]),
    plot=mesh_flag_right_jury_helper,
    orientation='vertical',
    zero_y=True,
)
# endregion

# endregion

# region not working Gearpod mesh 

# num_spanwise_gearpod_vlm_1 = 3
# num_spanwise_gearpod_vlm_2 = 9
# num_spanwise_gearpod_vlm = 2*((num_spanwise_gearpod_vlm_1-1) + (num_spanwise_gearpod_vlm_2-1)) + 1
# num_chordwise_gearpod_vlm = 5

# gearpod_strut_le_right = np.array([55.573, 12.461, -4.20])
# gearpod_strut_te_right = np.array([57.309, 12.641, -4.20])
# gearpod_strut_le_left = np.array([55.573, -12.641, -4.20])
# gearpod_strut_te_left = np.array([57.309, -12.641, -4.20])
# gearpod_le_right = np.array([54.807, 10.258, -4.796])
# gearpod_te_right = np.array([60.148, 10.258, -4.621])
# gearpod_le_left = np.array([54.807, -10.258, -4.796])
# gearpod_te_left = np.array([60.148, -10.258, -4.621])
# # gearpod_le_center = np.array([36.497, 0., -6.284])
# gearpod_le_center = np.array([35.80, 0., -5.25])
# gearpod_te_center = np.array([72.238, 0., -5.250])

# gearpod_leading_edge_points_le_1 = np.linspace(gearpod_strut_le_right, gearpod_le_right, num_spanwise_gearpod_vlm_1)[1:-1]
# gearpod_leading_edge_points_le_2 = np.linspace(gearpod_le_right, gearpod_le_center, num_spanwise_gearpod_vlm_2)[1:-1]
# gearpod_leading_edge_points_le_3 = np.linspace(gearpod_le_center, gearpod_le_left, num_spanwise_gearpod_vlm_2)[1:-1]
# gearpod_leading_edge_points_le_4 = np.linspace(gearpod_le_left, gearpod_strut_le_left, num_spanwise_gearpod_vlm_1)[1:-1]

# gearpod_leading_edge_points = np.vstack([gearpod_strut_le_right, gearpod_leading_edge_points_le_1, gearpod_le_right, gearpod_leading_edge_points_le_2, 
#                                         gearpod_le_center, gearpod_leading_edge_points_le_3, gearpod_le_left, gearpod_leading_edge_points_le_4, 
#                                         gearpod_strut_le_left])

# gearpod_trailing_edge_points_te_1 = np.linspace(gearpod_strut_te_right, gearpod_te_right, num_spanwise_gearpod_vlm_1)[1:-1]
# gearpod_trailing_edge_points_te_2 = np.linspace(gearpod_te_right, gearpod_te_center, num_spanwise_gearpod_vlm_2)[1:-1]
# gearpod_trailing_edge_points_te_3 = np.linspace(gearpod_te_center, gearpod_te_left, num_spanwise_gearpod_vlm_2)[1:-1]
# gearpod_trailing_edge_points_te_4 = np.linspace(gearpod_te_left, gearpod_strut_te_left, num_spanwise_gearpod_vlm_1)[1:-1]

# gearpod_trailing_edge_points = np.vstack([gearpod_strut_te_right, gearpod_trailing_edge_points_te_1, gearpod_te_right, gearpod_trailing_edge_points_te_2, 
#                                         gearpod_te_center, gearpod_trailing_edge_points_te_3, gearpod_te_left, gearpod_trailing_edge_points_te_4, 
#                                         gearpod_strut_te_left])

# gearpod_leading_edge = gearpod.project(gearpod_leading_edge_points, plot=do_plots)
# gearpod_trailing_edge = gearpod.project(gearpod_trailing_edge_points, plot=do_plots)

# gearpod_le_coord = geometry.evaluate(gearpod_leading_edge).reshape((-1, 3))
# gearpod_te_coord = geometry.evaluate(gearpod_trailing_edge).reshape((-1, 3))
# # A user can print the values of m3l variable by calling '.value'
# if print_coordinates:
#     print(gearpod_le_coord.value)
#     print(gearpod_te_coord.value)

# # Getting a linearly spaced (2-D) array between the leading and trailing edge
# gearpod_chord = m3l.linspace(gearpod_le_coord, gearpod_te_coord, num_chordwise_gearpod_vlm)

# # Projecting the 2-D array onto the upper and lower surface of the gearpod to get the camber surface mesh
# gearpod_upper_surface_wireframe_parametric = gearpod.project(gearpod_chord.value + np.array([0., 0., 1.]), direction=np.array([0., 0., 1.]), grid_search_density_parameter=25, plot=False)
# gearpod_lower_surface_wireframe_parametric = gearpod.project(gearpod_chord.value - np.array([0., 0., 1.]), direction=np.array([0., 0., -1.]), grid_search_density_parameter=25, plot=False)

# gearpod_upper_surface_wireframe = geometry.evaluate(gearpod_upper_surface_wireframe_parametric).reshape((num_chordwise_gearpod_vlm, num_spanwise_gearpod_vlm, 3))
# gearpod_lower_surface_wireframe = geometry.evaluate(gearpod_lower_surface_wireframe_parametric).reshape((num_chordwise_gearpod_vlm, num_spanwise_gearpod_vlm, 3))

# gearpod_camber_surface = m3l.linspace(gearpod_upper_surface_wireframe, gearpod_lower_surface_wireframe, 1)#.reshape((-1, 3))

# # Optionally, the resulting camber surface mesh can be plotted
# if mesh_flag_gearpod:
#     geometry.plot_meshes(meshes=gearpod_camber_surface, mesh_plot_types=['wireframe'], mesh_opacity=1., mesh_color='#F5F0E6')

# # endregion

# region not working Fuselage mesh 

# num_fuselage_len = 20
# num_fuselage_height = 4
# nose = np.array([0., 0., -2.8])
# rear = np.array([124.750, 0., 2.019])
# nose_points_parametric = Fuselage.project(nose, grid_search_density_parameter=20)
# rear_points_parametric = Fuselage.project(rear)

# nose_points_m3l = geometry.evaluate(nose_points_parametric)
# rear_points_m3l = geometry.evaluate(rear_points_parametric)

# fuselage_linspace = m3l.linspace(nose_points_m3l, rear_points_m3l, num_fuselage_len)

# fueslage_top_points_parametric = Fuselage.project(fuselage_linspace.value + np.array([0., 0., 3]), direction=np.array([0.,0.,-1.]), plot=False, grid_search_density_parameter=20)
# fueslage_bottom_points_parametric = Fuselage.project(fuselage_linspace.value - np.array([0., 0., 3]), direction=np.array([0.,0.,1.]), plot=False, grid_search_density_parameter=20)

# fueslage_top_points_m3l = geometry.evaluate(fueslage_top_points_parametric)
# fueslage_bottom_points_m3l = geometry.evaluate(fueslage_bottom_points_parametric)

# fuesleage_mesh = m3l.linspace(fueslage_top_points_m3l.reshape((-1, 3)), fueslage_bottom_points_m3l.reshape((-1, 3)),  int(num_fuselage_height + 1))
# fuesleage_mesh.description = 'zero_y'

# if mesh_flag_fuselage:
#     geometry.plot_meshes(meshes=fuesleage_mesh, mesh_plot_types=['wireframe'], mesh_opacity=1., mesh_color='#F5F0E6')
# # endregion

# if plot_unupdated_vlm_mesh: 
#     geometry.plot_meshes(meshes=[wing_meshes.vlm_mesh, htail_meshes.vlm_mesh, right_strut_meshes.vlm_mesh, left_strut_meshes.vlm_mesh, vtail_meshes.vlm_mesh, fuesleage_mesh, left_jury_meshes.vlm_mesh, right_jury_meshes.vlm_mesh, gearpod_camber_surface])

# endregion

# endregion

# endregion

# if True: 
#     # geometry.plot_meshes(meshes=[wing_meshes.vlm_mesh, htail_meshes.vlm_mesh])
#     # geometry.plot_meshes(meshes=[wing_meshes.vlm_mesh, htail_meshes.vlm_mesh, right_strut_meshes.vlm_mesh, left_strut_meshes.vlm_mesh])
#     geometry.plot_meshes(meshes=[right_strut_meshes.vlm_mesh, left_strut_meshes.vlm_mesh])
# exit()
# region beam mesh 
ft_to_m = 0.3048

# wing coordinates
wing00 = np.array([68.035, 85.291, 4.704]) # * ft2m # Right tip leading edge
wing01 = np.array([71.790, 85.291, 4.704]) # * ft2m # Right tip trailing edge
wing10 = np.array([47.231, 0.000, 6.937])  # * ft2m # Center Leading Edge
wing11 = np.array([57.953, 0.000, 6.566]) # * ft2m # Center Trailing edge
wing20 = np.array([68.035, -85.291, 4.704]) # * ft2m # Left tip leading edge
wing21 = np.array([71.790, -85.291, 4.704]) # * ft2m # Left tip trailing edge

# htail coordinates
htail00 = np.array([132.002-10.0, 19.217+4.5, 18.993+3.5]) # * ft2m # Right tip leading edge
htail01 = np.array([135.993, 19.217, 18.993]) # * ft2m # Right tip trailing edge
htail10 = np.array([122.905, 0.000, 20.000]) # * ft2m # Center Leading Edge
htail11 = np.array([134.308, 0.000, 20.000]) # * ft2m # Center Trailing edge
htail20 = np.array([132.002-10, -19.217-4.5, 18.993+3.5]) # * ft2m # Left tip leading edge
htail21 = np.array([135.993, -19.217, 18.993]) # * ft2m # Left tip trailing edge

# strut coordinates
strut00 = np.array([55.573, -12.641, -4.200]) # left leading 1
strut06 = np.array([61.090, -48.994, 5.763]) # left leading 7
strut10 = np.array([57.309, -12.641, -4.200]) # left trailing 1
strut16 = np.array([62.902, -48.994, 5.763]) # left trailing 7
strut20 = np.array([55.573, 12.641, -4.200]) # right leading 1
strut26 = np.array([61.090, 48.994, 5.763]) # right leading 7
strut30 = np.array([57.309, 12.641, -4.200]) # right trailing 1
strut36 = np.array([62.902, 48.994, 5.763]) # right trailing 7

# region wing beam mesh 
    
num_wing_beam = 21

# wing_beam_mesh_flag = True
# wing_box_beam_mesh = make_1d_box_beam_mesh(
#     geometry=geometry,
#     wing_component=wing,
#     num_beam_nodes=num_wing_beam,
#     te_right=wing01,
#     te_left=wing21,
#     te_center=wing11,
#     le_left=wing20,
#     le_right=wing00,
#     le_center=wing10,
#     beam_width=0.5,
#     node_center=0.5,
#     plot=wing_beam_mesh_flag,
#     le_interp = 'linear',
#     te_interp = 'linear',
# )

wing_beam_mesh_flag = False
# wing_beam_mesh_flag = True
wing_box_beam_mesh = make_1d_box_beam_mesh(
    geometry=geometry,
    wing_component=wing,
    num_beam_nodes=num_wing_beam,
    # te_right=np.array([73., 85., 5.507]),
    te_right=wing_te_right,
    # te_left=np.array([73., -85., 5.507]),
    te_left=wing_te_left,
    # te_center=np.array([57.953, 0.000, 6.566]),
    te_center=wing_te_center,
    # le_left=np.array([66., -85., 5.647]),
    le_left=wing_le_left,
    # le_right=np.array([66., 85., 5.647]),
    le_right=wing_le_right,
    # le_center=np.array([47.231, 0.000, 6.937]),
    le_center=wing_le_center,
    beam_width=0.5,
    node_center=0.35,
    front_spar_location = 0.25,
    rear_spar_location = 0.75, 
    plot=wing_beam_mesh_flag,
    le_interp='ellipse',
    te_interp='linear',
    horizontal = 'yes',
)

# endregion

# region strut beam mesh
num_strut_beam = 9
# num_strut_beam = 21
# left_strut_beam_mesh_flag = True
# region left strut beam mesh

left_strut_box_beam_mesh = make_1d_box_beam_mesh(
    geometry=geometry,
    wing_component=strut,
    num_beam_nodes=num_strut_beam,
    te_right=strut10,
    te_left=strut16,
    #te_center=vertex13_left_strut,
    le_left=strut06,
    le_right=strut00,
    #le_center=vertex03_left_strut,
    beam_width=0.5,
    node_center=0.35,
    front_spar_location = 0.25,
    rear_spar_location = 0.75, 
    plot=left_strut_beam_mesh_flag,
    le_interp = 'linear',
    te_interp = 'linear',
    horizontal = 'yes',
)
# endregion

# region right strut beam mesh
# right_strut_beam_mesh_flag = True
right_strut_box_beam_mesh = make_1d_box_beam_mesh(
    geometry=geometry,
    wing_component=strut,
    num_beam_nodes=num_strut_beam,
    te_right=strut30,
    te_left=strut36,
    #te_center=vertex13_left_strut,
    le_left=strut26,
    le_right=strut20,
    #le_center=vertex03_left_strut,
    beam_width=0.5,
    node_center=0.35,
    front_spar_location = 0.25,
    rear_spar_location = 0.75, 
    plot=right_strut_beam_mesh_flag,
    le_interp = 'linear',
    te_interp = 'linear',
    horizontal = 'yes',
)
# endregion
# endregion

# region jury beam mesh

# region left jury beam mesh
num_jury_beam = 3
# left_jury_beam_mesh_flag = True
left_jury_box_beam_mesh = make_1d_box_beam_mesh(
    geometry=geometry,
    wing_component=jury,
    num_beam_nodes=num_jury_beam,
    te_right=np.array([59.237, -30.818, 6.181]) ,
    te_left=np.array([59.676, -30.818, 0.732]) ,
    le_left=np.array([58.721, -30.818, 0.732,]) ,
    le_right=np.array([58.052, -30.818, 6.181]) ,
    beam_width=0.5,
    node_center=0.35,
    front_spar_location = 0.25,
    rear_spar_location = 0.75, 
    plot=left_jury_beam_mesh_flag,
    le_interp = 'linear',
    te_interp = 'linear',
    horizontal = 'no',
)
# endregion

# region right jury beam mesh
num_jury_beam = 3
# right_jury_beam_mesh_flag = True
right_jury_box_beam_mesh = make_1d_box_beam_mesh(
    geometry=geometry,
    wing_component=jury,
    num_beam_nodes=num_jury_beam,
    te_right=np.array([59.237, 30.818, 6.181]) ,
    te_left=np.array([59.676, 30.818, 0.732]) ,
    le_left=np.array([58.721, 30.818, 0.732,]) ,
    le_right=np.array([58.052, 30.818, 6.181]) ,
    beam_width=0.5,
    node_center=0.35,
    front_spar_location = 0.25,
    rear_spar_location = 0.75, 
    plot=right_jury_beam_mesh_flag,
    le_interp = 'linear',
    te_interp = 'linear',
    horizontal = 'no',
)
# endregion
# endregion

# endregion


def calculate_angle(point1, point2):
    # Calculate the vectors from the origin to each point
    vector1 = np.array(point1)
    vector2 = np.array(point2)
    
    # Calculate the dot product and magnitudes of the vectors
    dot_product = np.dot(vector1, vector2)
    magnitude1 = np.linalg.norm(vector1)
    magnitude2 = np.linalg.norm(vector2)
    
    # Calculate the angle in radians
    angle_radians = np.arccos(dot_product / (magnitude1 * magnitude2))
    
    # Convert the angle to degrees
    angle_degrees = np.degrees(angle_radians)
    
    return angle_degrees

# Example points
point1 = point40_wing
point2 = vertex05_left_strut

# Calculate the angle between the two points
angle_beta_not_m3l = calculate_angle(point1, point2)
angle_beta = system_model.create_input(name = 'angle_beta', shape = (1,), val = angle_beta_not_m3l)

FFD_working_old_5 = True # span and chord and twist working

if FFD_working_old_5: 
    
    # region True of False
    wing_plot = False
    htail_plot = False
    left_strut_plot = False
    right_strut_plot = False
    left_jury_plot = False
    right_jury_plot = False
    geometry_flag = False
    gearpod_plot = False

    # endregion

    # region Evaluations 

    wing_te_right_project = wing.project(wing_te_right)
    wing_te_left_project = wing.project(wing_te_left)
    wing_te_center_project = wing.project(wing_te_center)
    wing_le_left_project = wing.project(wing_le_left)
    wing_le_right_project = wing.project(wing_le_right)
    wing_le_center_project = wing.project(wing_le_center)
    wing_a_project = wing.project(point40_wing)
    wing_b_project = wing.project(point41_wing)
    wing_c_project = wing.project(point20_wing)
    wing_d_project = wing.project(point21_wing)

    left_strut_1 = strut_left.project(vertex06_left_strut)
    left_strut_2 = strut_left.project(vertex00_left_strut)
    left_strut_3 = strut_left.project(vertex16_left_strut)

    right_strut_1 = strut_right.project(vertex06_right_strut)
    right_strut_2 = strut_right.project(vertex00_right_strut)
    right_strut_3 = strut_right.project(vertex16_right_strut)

    left_jury_le_left=np.array([58.721, -30.818, 0.732,]) 
    left_jury_le_right=np.array([58.052, -30.818, 6.181])
    left_jury_bot = jury_left.project(left_jury_le_left)
    left_jury_top = jury_left.project(left_jury_le_right)

    right_jury_le_left=np.array([58.721, 30.818, 0.732,]) 
    right_jury_le_right=np.array([58.052, 30.818, 6.181])
    right_jury_bot = jury_right.project(right_jury_le_left)
    right_jury_top = jury_right.project(right_jury_le_right)

    #wing and fuselage connection
    wing_fuselage_connection_point = np.array([52.548, 0., 6.268])
    # fusleage_projection_on_wing = Fuselage.project(wing_le_center)
    # wing_projection_on_fuselage = wing.project(wing_le_center)
    fusleage_projection_on_wing = Fuselage.project(wing_fuselage_connection_point)
    wing_projection_on_fuselage = wing.project(wing_fuselage_connection_point)

    # wing and strut connection
    # left -ve y
    # wing_left_strut_connection_point = np.array([61.661, -48.994, 5.927])
    wing_left_strut_connection_point = np.array([62.309, -49.280, 6.095])
    wing_projection_on_left_strut = wing.project(wing_left_strut_connection_point)
    left_strut_projection_on_wing = strut_left.project(wing_left_strut_connection_point)
    #right +ve y
    # wing_right_strut_connection_point = np.array([61.661, 48.994, 5.927])
    wing_right_strut_connection_point = np.array([62.309, 49.280, 6.095])
    wing_projection_on_right_strut = wing.project(wing_right_strut_connection_point)
    right_strut_projection_on_wing = strut_right.project(wing_right_strut_connection_point)

    #jury and wing connection
    # left
    wing_left_jury_connection_point = np.array([58.633, -30.818, 6.181])
    wing_projection_on_left_jury = wing.project(wing_left_jury_connection_point)
    left_jury_projection_on_wing = jury_left.project(wing_left_jury_connection_point)
    # right 
    wing_right_jury_connection_point = np.array([58.633, 30.818, 6.181])
    wing_projection_on_right_jury = wing.project(wing_right_jury_connection_point)
    right_jury_projection_on_wing = jury_right.project(wing_right_jury_connection_point)

    # jury and strut connection
    # left
    left_jury_left_strut_connection_point = np.array([59.190, -30.818, 0.732])
    left_jury_projection_on_left_strut = jury_left.project(left_jury_left_strut_connection_point)
    left_strut_projection_on_left_jury = strut_left.project(left_jury_left_strut_connection_point)
    #right
    right_jury_right_strut_connection_point = np.array([59.190, 30.818, 0.732])
    right_jury_projection_on_right_strut = jury_right.project(right_jury_right_strut_connection_point)
    right_strut_projection_on_right_jury = strut_right.project(right_jury_right_strut_connection_point)

    # gearpod and strut
    # left
    # gearpod_left_strut_connection_point = np.array([55.573, -12.641, -4.200])
    gearpod_left_strut_connection_point = np.array([56.425, -12.641, -4.200])
    gearpod_projection_on_left_strut = gearpod.project(gearpod_left_strut_connection_point)
    # left_strut_projection_on_gearpod = strut_left.project(gearpod_left_strut_connection_point)
    left_strut_projection_on_gearpod = strut.project(gearpod_left_strut_connection_point)
    # right
    gearpod_right_strut_connection_point = np.array([56.425, 12.641, -4.200])
    gearpod_projection_on_right_strut = gearpod.project(gearpod_right_strut_connection_point)
    # right_strut_projection_on_gearpod = strut_right.project(gearpod_right_strut_connection_point)
    right_strut_projection_on_gearpod = strut.project(gearpod_right_strut_connection_point)

    # endregion

    constant_b_spline_curve_1_dof_space =  bsp.BSplineSpace(name='constant_b_spline_curve_1_dof_space', order=1, parametric_coefficients_shape=(1,))
    linear_b_spline_curve_2_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_2_dof_space', order=2, parametric_coefficients_shape=(2,))
    linear_b_spline_curve_3_dof_space = bsp.BSplineSpace(name='linear_b_spline_curve_3_dof_space', order=2, parametric_coefficients_shape=(3,))
    cubic_b_spline_curve_5_dof_space = bsp.BSplineSpace(name='cubic_b_spline_curve_5_dof_space', order=4, parametric_coefficients_shape=(5,))
    from lsdo_geo.core.parameterization.parameterization_solver import ParameterizationSolver
    from lsdo_geo.core.parameterization.volume_sectional_parameterization import VolumeSectionalParameterization
    tbw_param_solver = ParameterizationSolver()

    # region Wing

    # region Volume
    import numpy as np

    # Define the corner points 
    corner_00 = np.array([72.00, -85.50, 4.9])
    corner_01 = np.array([72.00, -85.50, 4.5])
    corner_02 = np.array([68.00, -85.50, 4.9])
    corner_03 = np.array([68.00, -85.50, 4.5])
    corner_10 = np.array([67.50, -49.280, 6.4])
    corner_11 = np.array([67.50, -49.280, 5.7])
    corner_12 = np.array([57.00, -49.280, 6.4])
    corner_13 = np.array([57.00, -49.280, 5.7])
    corner_20 = np.array([58.00, 0.0, 7.4])
    corner_21 = np.array([58.00, 0.0, 6.4])
    corner_22 = np.array([47.1, 0., 7.4])
    corner_23 = np.array([47.1, 0., 6.4])
    corner_30 = np.array([67.50, 49.280, 6.4])
    corner_31 = np.array([67.50, 49.280, 5.7])
    corner_32 = np.array([57.00, 49.280, 6.4])
    corner_33 = np.array([57.00, 49.280, 5.7])
    corner_40 = np.array([72.00, 85.50, 4.9])
    corner_41 = np.array([72.00, 85.50, 4.5])
    corner_42 = np.array([68.00, 85.50, 4.9])
    corner_43 = np.array([68.00, 85.50, 4.5])

#     corner_00 = np.array([72.00, -85.50, 8.0]) # trailing
#     corner_01 = np.array([72.00, -85.50, 4.0])
#     corner_02 = np.array([68.00, -85.50, 8.0]) # leading
#     corner_03 = np.array([68.00, -85.50, 4.0])
#     corner_10 = np.array([67.50, -49.280, 8.0])
#     corner_11 = np.array([67.50, -49.280, 4.0])
#     corner_12 = np.array([57.0, -49.280, 8.0])
#     corner_13 = np.array([57.0, -49.280, 4.0])
#     corner_20 = np.array([58.00, 0.0, 8.0])
#     corner_21 = np.array([58.00, 0.0, 4.0])
#     corner_22 = np.array([47.1, 0., 8.0])
#     corner_23 = np.array([47.1, 0., 4.0])
#     corner_30 = np.array([67.50, 42.646, 8.0])
#     corner_31 = np.array([67.50, 42.646, 4.0])
#     corner_32 = np.array([57.0, 42.646, 8.0])
#     corner_33 = np.array([57.00, 42.646, 4.0])
#     corner_40 = np.array([72.00, 85.50, 8.0])
#     corner_41 = np.array([72.00, 85.50, 4.0])
#     corner_42 = np.array([68.00, 85.50, 8.0])
#     corner_43 = np.array([68.00, 85.50, 4.0])

    # get the x,y and z direction points
    points = np.array([corner_00,corner_01,corner_02,corner_03,corner_10,corner_11,corner_12,corner_13,corner_20,corner_21,corner_22,corner_23,
                    corner_30,corner_31,corner_32,corner_33,corner_40,corner_41,corner_42,corner_43])

    x_coordinates = points[:,0]
    y_coordinates = points[:,1]
    z_coordinates = points[:,2]
    # print(x_coordinates)
    # print(y_coordinates)
    # print(z_coordinates)
    # print(x_coordinates.shape)
    # print(y_coordinates.shape)
    # print(z_coordinates.shape)
    
    # get the 10 lines that go along the z-axis
    line_1 = np.vstack((corner_00, corner_01))
    line_2 = np.vstack((corner_10, corner_11))
    line_3 = np.vstack((corner_20, corner_21))
    line_4 = np.vstack((corner_30, corner_31))
    line_5 = np.vstack((corner_40, corner_41))
    line_6 = np.vstack((corner_02, corner_03))
    line_7 = np.vstack((corner_12, corner_13))
    line_8 = np.vstack((corner_22, corner_23))
    line_9 = np.vstack((corner_32, corner_33))
    line_10 = np.vstack((corner_42, corner_43))

    # 10 lines into two surfaces
    all_lines = np.concatenate((line_1, line_2, line_3, line_4, line_5), axis=0)
    all_lines_1 = np.concatenate((line_6, line_7, line_8, line_9, line_10), axis=0)

    # two surfaces into 1 volume
    volume = np.stack((all_lines, all_lines_1), axis=0)
    # print(volume.shape)
    volume_1 = volume.reshape(2,5,2,3)
    # print(volume_1.shape)
    # print(volume_1)
    # endregion 
         

    # wing_ffd_block = lg.construct_ffd_block_around_entities(name='wing_ffd_block', entities=wing, num_coefficients=(2, 11, 2), order=(2, 2, 2))
    wing_ffd_block = lg.construct_ffd_block_from_corners(name='wing_ffd_block', entities=wing, corners=volume_1, 
                                                         num_coefficients=(2, 2, 2), order=(2, 2, 2))

    wing_ffd_block_sect_param = VolumeSectionalParameterization(name='wing_ffd_sectional_parameterization', 
                                                                principal_parametric_dimension=1, 
                                                                parameterized_points=wing_ffd_block.coefficients,
                                                                parameterized_points_shape=wing_ffd_block.coefficients_shape)
    
    wing_ffd_block_sect_param.add_sectional_translation(name='wing_span_stretch', axis=1)
    wing_ffd_block_sect_param.add_sectional_stretch(name='wing_chord_stretch', axis=0)

    # wing_span_stretch_coefficients = m3l.Variable(name= 'wing_span_stretch_coefficients', shape=(2, ), value=np.array([0.,0.]))
    wing_span_stretch_coefficients = system_model.create_input(name= 'wing_span_stretch_coefficients', shape=(2, ), val=np.array([0.,0.]))
    wing_span_strech_b_spline = bsp.BSpline(name='wing_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=wing_span_stretch_coefficients, num_physical_dimensions=1)
        
    # wing_chord_stretch_coefficients = m3l.Variable(name= 'wing_chord_stretch_coefficients', shape=(5, ), value=np.array([0., 0., 0., 0., 0.]))
    wing_chord_stretch_coefficients = system_model.create_input(name= 'wing_chord_stretch_coefficients', shape=(5, ), val=np.array([0., 0., 0., 0., 0.]))
    wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_b_spline', space=cubic_b_spline_curve_5_dof_space, 
                                              coefficients=wing_chord_stretch_coefficients, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sect_param.num_sections).reshape((-1,1))
    wing_span_stretch = wing_span_strech_b_spline.evaluate(section_parametric_coordinates)
    wing_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'wing_span_stretch': wing_span_stretch,
            'wing_chord_stretch': wing_chord_stretch,
    }
    
    wing_ffd_block_coefficients = wing_ffd_block_sect_param.evaluate(sectional_parameters, plot=wing_plot)
    wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=wing_plot)
    # endregion

    # region strut 

    # region left strut try 
    left_strut_ffd_block = lg.construct_ffd_block_around_entities(name='left_strut_ffd_block', entities=strut_left, num_coefficients=(2, 5, 2), order=(2, 2, 2))
    
    left_strut_ffd_block_sect_param = VolumeSectionalParameterization(name='left_strut_ffd_sectional_parameterization', 
                                                                principal_parametric_dimension=1, 
                                                                parameterized_points=left_strut_ffd_block.coefficients,
                                                                parameterized_points_shape=left_strut_ffd_block.coefficients_shape)

    left_strut_ffd_block_sect_param.add_sectional_translation(name='left_strut_span_stretch', axis=1)
    left_strut_ffd_block_sect_param.add_sectional_translation(name='left_strut_translation_x', axis=0)
    left_strut_ffd_block_sect_param.add_sectional_translation(name='left_strut_translation_z', axis=2)

    # strut_span_stretch_coefficients = m3l.Variable(name= 'strut_span_stretch_coefficients', shape=(2, ), value=np.array([0.,0.]))
    left_strut_span_stretch_coefficients = system_model.create_input(name= 'left_strut_span_stretch_coefficients', shape=(2, ), val=np.array([0.,0.]))
    left_strut_span_strech_b_spline = bsp.BSpline(name='left_strut_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=left_strut_span_stretch_coefficients, num_physical_dimensions=1)

    left_strut_translation_x_coefficients = system_model.create_input('left_strut_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    left_strut_translation_z_coefficients = system_model.create_input('left_strut_translation_z_coefficients', shape=(1, ), val=np.array([0]))
    
    left_strut_ffd_block_translation_x_b_spline = bsp.BSpline(name='left_strut_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_strut_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    left_strut_ffd_block_translation_z_b_spline = bsp.BSpline(name='left_strut_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_strut_translation_z_coefficients, 
                                                        num_physical_dimensions=1)
        
    section_parametric_coordinates = np.linspace(0., 1., left_strut_ffd_block_sect_param.num_sections).reshape((-1,1))
    left_strut_translation_x = left_strut_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    left_strut_translation_z = left_strut_ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)
    left_strut_span_stretch = left_strut_span_strech_b_spline.evaluate(section_parametric_coordinates)
    
    # section_parametric_coordinates = np.linspace(0., 1., left_strut_ffd_block_sect_param.num_sections).reshape((-1,1))
    # left_strut_span_stretch = left_strut_span_strech_b_spline.evaluate(section_parametric_coordinates)
    # left_strut_chord_stretch = left_strut_chord_stretch_b_spline.evaluate(section_parametric_coordinates)

    # sectional_parameters = {
    #         'left_strut_span_stretch': left_strut_span_stretch,
    #         # 'left_strut_chord_stretch': left_strut_chord_stretch,
    # }
    

    sectional_parameters = {
            'left_strut_translation_x': left_strut_translation_x,
            'left_strut_translation_z': left_strut_translation_z,
            'left_strut_span_stretch': left_strut_span_stretch,
    }

    left_strut_ffd_block_coefficients = left_strut_ffd_block_sect_param.evaluate(sectional_parameters, plot=left_strut_plot)
    left_strut_coefficients = left_strut_ffd_block.evaluate(left_strut_ffd_block_coefficients, plot=left_strut_plot)

    # endregion

    # region right strut new

    right_strut_ffd_block = lg.construct_ffd_block_around_entities(name='right_strut_ffd_block', entities=strut_right, num_coefficients=(2, 5, 2), order=(2, 2, 2))
    
    right_strut_ffd_block_sect_param = VolumeSectionalParameterization(name='right_strut_ffd_sectional_parameterization', 
                                                                principal_parametric_dimension=1, 
                                                                parameterized_points=right_strut_ffd_block.coefficients,
                                                                parameterized_points_shape=right_strut_ffd_block.coefficients_shape)

    right_strut_ffd_block_sect_param.add_sectional_translation(name='right_strut_span_stretch', axis=1)
    right_strut_ffd_block_sect_param.add_sectional_translation(name='right_strut_translation_x', axis=0)
    right_strut_ffd_block_sect_param.add_sectional_translation(name='right_strut_translation_z', axis=2)

    # strut_span_stretch_coefficients = m3l.Variable(name= 'strut_span_stretch_coefficients', shape=(2, ), value=np.array([0.,0.]))
    right_strut_span_stretch_coefficients = system_model.create_input(name= 'right_strut_span_stretch_coefficients', shape=(2, ), val=np.array([0.,0.]))
    right_strut_span_strech_b_spline = bsp.BSpline(name='right_strut_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=right_strut_span_stretch_coefficients, num_physical_dimensions=1)

    right_strut_translation_x_coefficients = system_model.create_input('right_strut_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    right_strut_translation_z_coefficients = system_model.create_input('right_strut_translation_z_coefficients', shape=(1, ), val=np.array([0]))
    
    right_strut_ffd_block_translation_x_b_spline = bsp.BSpline(name='right_strut_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_strut_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    right_strut_ffd_block_translation_z_b_spline = bsp.BSpline(name='right_strut_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_strut_translation_z_coefficients, 
                                                        num_physical_dimensions=1)
        
    section_parametric_coordinates = np.linspace(0., 1., right_strut_ffd_block_sect_param.num_sections).reshape((-1,1))
    right_strut_translation_x = right_strut_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    right_strut_translation_z = right_strut_ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)
    right_strut_span_stretch = right_strut_span_strech_b_spline.evaluate(section_parametric_coordinates)
    
    # section_parametric_coordinates = np.linspace(0., 1., right_strut_ffd_block_sect_param.num_sections).reshape((-1,1))
    # right_strut_span_stretch = right_strut_span_strech_b_spline.evaluate(section_parametric_coordinates)
    # right_strut_chord_stretch = right_strut_chord_stretch_b_spline.evaluate(section_parametric_coordinates)

    # sectional_parameters = {
    #         'right_strut_span_stretch': right_strut_span_stretch,
    #         # 'right_strut_chord_stretch': right_strut_chord_stretch,
    # }
    

    sectional_parameters = {
            'right_strut_translation_x': right_strut_translation_x,
            'right_strut_translation_z': right_strut_translation_z,
            'right_strut_span_stretch': right_strut_span_stretch,
    }

    right_strut_ffd_block_coefficients = right_strut_ffd_block_sect_param.evaluate(sectional_parameters, plot=right_strut_plot)
    right_strut_coefficients = right_strut_ffd_block.evaluate(right_strut_ffd_block_coefficients, plot=right_strut_plot)

    # endregion      

    # endregion

    # region Jury

    # region left Jury 

    left_jury_ffd_block = lg.construct_ffd_block_around_entities(name='left_jury_ffd_block', entities=jury_left, num_coefficients=(2, 2, 5), order=(2, 2, 2))
    
    left_jury_ffd_block_sect_param = VolumeSectionalParameterization(name='left_jury_ffd_sectional_parameterization', 
                                                                principal_parametric_dimension=1, 
                                                                parameterized_points=left_jury_ffd_block.coefficients,
                                                                parameterized_points_shape=left_jury_ffd_block.coefficients_shape)

    left_jury_ffd_block_sect_param.add_sectional_translation(name='left_jury_translation_x', axis= 0)
    left_jury_ffd_block_sect_param.add_sectional_translation(name='left_jury_translation_y', axis= 1)
    left_jury_ffd_block_sect_param.add_sectional_stretch(name='left_jury_stretch_z', axis=2)


    left_jury_translation_x_coefficients = system_model.create_input('left_jury_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    left_jury_translation_y_coefficients = system_model.create_input('left_jury_translation_y_coefficients', shape=(1, ), val=np.array([0]))
    left_jury_stretch_z_coefficients = system_model.create_input('left_jury_stretch_z_coefficients', shape=(1, ), val=np.array([0]))

    left_jury_ffd_block_translation_x_b_spline = bsp.BSpline(name='left_jury_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_jury_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    left_jury_ffd_block_translation_y_b_spline = bsp.BSpline(name='left_jury_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_jury_translation_y_coefficients, 
                                                        num_physical_dimensions=1)
        
    left_jury_ffd_block_stretch_z_b_spline = bsp.BSpline(name='left_jury_stretch_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_jury_stretch_z_coefficients, 
                                                        num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., left_jury_ffd_block_sect_param.num_sections).reshape((-1,1))
    left_jury_translation_x = left_jury_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    left_jury_translation_y = left_jury_ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
    left_jury_stretch_z = left_jury_ffd_block_stretch_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'left_jury_translation_x': left_jury_translation_x,
            'left_jury_translation_y': left_jury_translation_y,
            'left_jury_stretch_z': left_jury_stretch_z,
    }
    
    left_jury_ffd_block_coefficients = left_jury_ffd_block_sect_param.evaluate(sectional_parameters, plot=left_jury_plot)
    left_jury_coefficients = left_jury_ffd_block.evaluate(left_jury_ffd_block_coefficients, plot=left_jury_plot)

    # endregion

    # region right Jury 

    right_jury_ffd_block = lg.construct_ffd_block_around_entities(name='right_jury_ffd_block', entities=jury_right, num_coefficients=(2, 2, 5), order=(2, 2, 2))
    
    right_jury_ffd_block_sect_param = VolumeSectionalParameterization(name='right_jury_ffd_sectional_parameterization', 
                                                                principal_parametric_dimension=1, 
                                                                parameterized_points=right_jury_ffd_block.coefficients,
                                                                parameterized_points_shape=right_jury_ffd_block.coefficients_shape)

    right_jury_ffd_block_sect_param.add_sectional_translation(name='right_jury_translation_x', axis=0)
    right_jury_ffd_block_sect_param.add_sectional_translation(name='right_jury_translation_y', axis=1)
    right_jury_ffd_block_sect_param.add_sectional_stretch(name='right_jury_stretch_z', axis=2)

    right_jury_translation_x_coefficients = system_model.create_input('right_jury_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    right_jury_translation_y_coefficients = system_model.create_input('right_jury_translation_y_coefficients', shape=(1, ), val=np.array([0]))
    right_jury_stretch_z_coefficients = system_model.create_input('right_jury_stretch_z_coefficients', shape=(1, ), val=np.array([0]))

    right_jury_ffd_block_translation_x_b_spline = bsp.BSpline(name='right_jury_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_jury_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    right_jury_ffd_block_translation_y_b_spline = bsp.BSpline(name='right_jury_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_jury_translation_y_coefficients, 
                                                        num_physical_dimensions=1)
                
    right_jury_ffd_block_stretch_z_b_spline = bsp.BSpline(name='right_jury_stretch_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_jury_stretch_z_coefficients, 
                                                        num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., right_jury_ffd_block_sect_param.num_sections).reshape((-1,1))
    right_jury_translation_x = right_jury_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    right_jury_translation_y = right_jury_ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
    right_jury_stretch_z = right_jury_ffd_block_stretch_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'right_jury_translation_x': right_jury_translation_x,
            'right_jury_translation_y': right_jury_translation_y,
            'right_jury_stretch_z': right_jury_stretch_z,
    }
    
    right_jury_ffd_block_coefficients = right_jury_ffd_block_sect_param.evaluate(sectional_parameters, plot=right_jury_plot)
    right_jury_coefficients = right_jury_ffd_block.evaluate(right_jury_ffd_block_coefficients, plot=right_jury_plot)    

    # endregion

    # endregion

    # region gearpod

    gearpod_ffd_block = lg.construct_ffd_block_around_entities(name='gearpod_ffd_block', entities=gearpod, num_coefficients=(2, 3, 2), order=(2, 2, 2))

    gearpod_ffd_block_sect_param = VolumeSectionalParameterization(name='gearpod_ffd_sectional_parameterization', 
                                                                principal_parametric_dimension=1, 
                                                                parameterized_points=gearpod_ffd_block.coefficients,
                                                                parameterized_points_shape=gearpod_ffd_block.coefficients_shape)
    
    gearpod_ffd_block_sect_param.add_sectional_translation(name='gearpod_translation_y', axis=1)
    # gearpod_ffd_block_sect_param.add_sectional_stretch(name='gearpod_chord_stretch', axis=0)
    gearpod_ffd_block_sect_param.add_sectional_translation(name='gearpod_translation_x', axis=0)
    gearpod_ffd_block_sect_param.add_sectional_translation(name='gearpod_translation_z', axis=2)

    # gearpod_span_stretch_coefficients = m3l.Variable(name= 'gearpod_span_stretch_coefficients', shape=(2, ), value=np.array([0.,0.]))
    gearpod_translation_y_coefficients = system_model.create_input(name= 'gearpod_translation_y_coefficients', shape=(2, ), val=np.array([0.,0.]))
    gearpod_span_strech_b_spline = bsp.BSpline(name='gearpod_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=gearpod_translation_y_coefficients, num_physical_dimensions=1)
        
    # gearpod_chord_stretch_coefficients = m3l.Variable(name= 'gearpod_chord_stretch_coefficients', shape=(5, ), value=np.array([0., 0., 0., 0., 0.]))
    # gearpod_chord_stretch_coefficients = system_model.create_input(name= 'gearpod_chord_stretch_coefficients', shape=(5, ), val=np.array([0., 0., 0., 0., 0.]))
    # gearpod_chord_stretch_b_spline = bsp.BSpline(name='gearpod_chord_b_spline', space=cubic_b_spline_curve_5_dof_space, 
    #                                           coefficients=gearpod_chord_stretch_coefficients, num_physical_dimensions=1)

    gearpod_translation_x_coefficients = system_model.create_input('gearpod_translation_x_coefficients', shape=(1, ), val=np.array([0]))
    gearpod_translation_z_coefficients = system_model.create_input('gearpod_translation_z_coefficients', shape=(1, ), val=np.array([0]))
    
    gearpod_ffd_block_translation_x_b_spline = bsp.BSpline(name='gearpod_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=gearpod_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    gearpod_ffd_block_translation_z_b_spline = bsp.BSpline(name='gearpod_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=gearpod_translation_z_coefficients, 
                                                        num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., gearpod_ffd_block_sect_param.num_sections).reshape((-1,1))
    gearpod_translation_y = gearpod_span_strech_b_spline.evaluate(section_parametric_coordinates)
#     gearpod_chord_stretch = gearpod_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
    gearpod_translation_x = gearpod_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    gearpod_translation_z = gearpod_ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'gearpod_translation_y': gearpod_translation_y,
        #     'gearpod_chord_stretch': gearpod_chord_stretch,
        'gearpod_translation_x' : gearpod_translation_x,
        'gearpod_translation_z' : gearpod_translation_z,
    }
    # gearpod_plot = True
    gearpod_ffd_block_coefficients = gearpod_ffd_block_sect_param.evaluate(sectional_parameters, plot=gearpod_plot)
    gearpod_coefficients = gearpod_ffd_block.evaluate(gearpod_ffd_block_coefficients, plot=gearpod_plot)

    # endregion

    coefficients_list = []
    coefficients_list.append(wing_coefficients)
    coefficients_list.append(left_strut_coefficients)
    coefficients_list.append(right_strut_coefficients)
    coefficients_list.append(left_jury_coefficients)
    coefficients_list.append(right_jury_coefficients)
    coefficients_list.append(gearpod_coefficients)

    b_spline_names_list = []
    b_spline_names_list.append(wing.b_spline_names)
    b_spline_names_list.append(strut_left.b_spline_names)
    b_spline_names_list.append(strut_right.b_spline_names)
    b_spline_names_list.append(jury_left.b_spline_names)
    b_spline_names_list.append(jury_right.b_spline_names)
    b_spline_names_list.append(gearpod.b_spline_names)

    geometry.assign_coefficients(coefficients=wing_coefficients, b_spline_names=wing.b_spline_names)
    geometry.assign_coefficients(coefficients=left_strut_coefficients, b_spline_names=strut_left.b_spline_names)
    geometry.assign_coefficients(coefficients=right_strut_coefficients, b_spline_names=strut_right.b_spline_names)
    geometry.assign_coefficients(coefficients=left_jury_coefficients, b_spline_names=jury_left.b_spline_names)
    geometry.assign_coefficients(coefficients=right_jury_coefficients, b_spline_names=jury_right.b_spline_names)
    geometry.assign_coefficients(coefficients=gearpod_coefficients, b_spline_names=gearpod.b_spline_names)

    geometry.assign_coefficients(coefficients=coefficients_list, b_spline_names=b_spline_names_list)
    # geometry_flag = True
    
    if geometry_flag:
        geometry.plot()

    # region Declaring states and input to the inner optimization

    wing_te_right_m3l = geometry.evaluate(wing_te_right_project)
    wing_te_left_m3l = geometry.evaluate(wing_te_left_project)
    wing_te_center_m3l = geometry.evaluate(wing_te_center_project)
    wing_le_left_m3l = geometry.evaluate(wing_le_left_project)
    wing_le_right_m3l = geometry.evaluate(wing_le_right_project)
    wing_le_center_m3l = geometry.evaluate(wing_le_center_project)
    wing_a_m3l = geometry.evaluate(wing_a_project)
    wing_b_m3l = geometry.evaluate(wing_b_project)
    wing_c_m3l = geometry.evaluate(wing_c_project)
    wing_d_m3l = geometry.evaluate(wing_d_project)

    wing_span = m3l.norm(wing_te_right_m3l - wing_te_left_m3l)
    root_chord = m3l.norm(wing_te_center_m3l-wing_le_center_m3l)
    tip_chord_left = m3l.norm(wing_te_left_m3l-wing_le_left_m3l)
    tip_chord_right = m3l.norm(wing_te_right_m3l-wing_le_right_m3l)
    midpoint_wing_left = m3l.norm(wing_a_m3l - wing_b_m3l)
    midpoint_wing_right = m3l.norm(wing_c_m3l - wing_d_m3l)

    left_strut_1_m3l = geometry.evaluate(left_strut_1)
    left_strut_2_m3l = geometry.evaluate(left_strut_2)
    # left_strut_3_m3l = geometry.evaluate(left_strut_3)

    right_strut_1_m3l = geometry.evaluate(right_strut_1)
    right_strut_2_m3l = geometry.evaluate(right_strut_2)
    # right_strut_3_m3l = geometry.evaluate(right_strut_3)

    left_strut_span = m3l.norm(left_strut_1_m3l - left_strut_2_m3l)
    # left_strut_chord = m3l.norm(left_strut_1_m3l - left_strut_3_m3l)
    right_strut_span = m3l.norm(right_strut_1_m3l - right_strut_2_m3l)
    # right_strut_chord = m3l.norm(right_strut_1_m3l - right_strut_3_m3l)

    left_jury_bot_m3l = geometry.evaluate(left_jury_bot)
    left_jury_top_m3l = geometry.evaluate(left_jury_top)
    left_jury_height = m3l.norm(left_jury_top_m3l - left_jury_bot_m3l)

    right_jury_bot_m3l = geometry.evaluate(right_jury_bot)
    right_jury_top_m3l = geometry.evaluate(right_jury_top)
    right_jury_height = m3l.norm(right_jury_top_m3l - right_jury_bot_m3l)

    # wing and fuselage
    wing_point_on_fuselage_m3l = geometry.evaluate(fusleage_projection_on_wing)
    wing_point_on_wing_fuselage_m3l = geometry.evaluate(wing_projection_on_fuselage)
    wing_fuselage_connection = m3l.norm(wing_point_on_fuselage_m3l-wing_point_on_wing_fuselage_m3l)
    wing_fuselage_connection_m3l = m3l.subtract(wing_point_on_fuselage_m3l, wing_point_on_wing_fuselage_m3l)

    # gearpod and strut
    # left
    left_strut_point_on_gearpod_m3l = geometry.evaluate(gearpod_projection_on_left_strut)
    gearpod_point_on_left_strut_m3l = geometry.evaluate(left_strut_projection_on_gearpod)
    gearpod_left_strut_connection = m3l.norm(left_strut_point_on_gearpod_m3l-gearpod_point_on_left_strut_m3l)
    gearpod_left_strut_connection_m3l = m3l.subtract(left_strut_point_on_gearpod_m3l,gearpod_point_on_left_strut_m3l)
    # right
    right_strut_point_on_gearpod_m3l = geometry.evaluate(gearpod_projection_on_right_strut)
    gearpod_point_on_right_strut_m3l = geometry.evaluate(right_strut_projection_on_gearpod)
    gearpod_right_strut_connection = m3l.norm(right_strut_point_on_gearpod_m3l-gearpod_point_on_right_strut_m3l)
    gearpod_right_strut_connection_m3l = m3l.subtract(right_strut_point_on_gearpod_m3l,gearpod_point_on_right_strut_m3l)

    # right_strut_point_on_gearpod_m3l = geometry.evaluate(gearpod_projection_on_right_strut)
    # gearpod_point_on_right_strut_m3l = geometry.evaluate(right_strut_projection_on_gearpod)
    # gearpod_right_strut_connection = m3l.norm(right_strut_point_on_gearpod_m3l-gearpod_point_on_right_strut_m3l)
    # gearpod_right_strut_connection_m3l = m3l.subtract(right_strut_point_on_gearpod_m3l,gearpod_point_on_right_strut_m3l)

    # wing and strut 
    # left
    wing_point_on_left_strut_m3l = geometry.evaluate(wing_projection_on_left_strut)
    left_strut_point_on_wing_m3l = geometry.evaluate(left_strut_projection_on_wing)
    # wing_left_strut_connection = m3l.subtract(wing_point_on_left_strut_m3l,left_strut_point_on_wing_m3l)
    wing_left_strut_connection = m3l.norm(wing_point_on_left_strut_m3l - left_strut_point_on_wing_m3l)
    wing_left_strut_connection_m3l = m3l.subtract(wing_point_on_left_strut_m3l,left_strut_point_on_wing_m3l)

    # right
    wing_point_on_right_strut_m3l = geometry.evaluate(wing_projection_on_right_strut)
    right_strut_point_on_wing_m3l = geometry.evaluate(right_strut_projection_on_wing)
    # wing_right_strut_connection_m3l = m3l.subtract(wing_point_on_right_strut_m3l, right_strut_point_on_wing_m3l)
    wing_right_strut_connection = m3l.norm(wing_point_on_right_strut_m3l - right_strut_point_on_wing_m3l)
    wing_right_strut_connection_m3l = m3l.subtract(wing_point_on_right_strut_m3l, right_strut_point_on_wing_m3l)

    # wing and jury
    # left
    wing_point_on_left_jury_m3l = geometry.evaluate(wing_projection_on_left_jury)
    left_jury_point_on_wing_m3l = geometry.evaluate(left_jury_projection_on_wing)
    wing_left_jury_connection_m3l = m3l.subtract(wing_point_on_left_jury_m3l, left_jury_point_on_wing_m3l)
    wing_left_jury_connection = m3l.norm(wing_point_on_left_jury_m3l - left_jury_point_on_wing_m3l)
    # jury
    wing_point_on_right_jury_m3l = geometry.evaluate(wing_projection_on_right_jury)
    right_jury_point_on_wing_m3l = geometry.evaluate(right_jury_projection_on_wing)
    wing_right_jury_connection_m3l = m3l.subtract(wing_point_on_right_jury_m3l, right_jury_point_on_wing_m3l)
    wing_right_jury_connection = m3l.norm(wing_point_on_right_jury_m3l - right_jury_point_on_wing_m3l)
    
    # strut and jury
    # left 
    left_strut_point_on_left_jury_m3l = geometry.evaluate(left_strut_projection_on_left_jury)
    left_jury_point_on_left_strut_m3l = geometry.evaluate(left_jury_projection_on_left_strut)
    left_strut_left_jury_connection_m3l = m3l.subtract(left_strut_point_on_left_jury_m3l, left_jury_point_on_left_strut_m3l)
    left_strut_left_jury_connection = m3l.norm(left_strut_point_on_left_jury_m3l - left_jury_point_on_left_strut_m3l)
    # right
    right_strut_point_on_right_jury_m3l = geometry.evaluate(right_strut_projection_on_right_jury)
    right_jury_point_on_right_strut_m3l = geometry.evaluate(right_jury_projection_on_right_strut)
    right_strut_right_jury_connection_m3l = m3l.subtract(right_strut_point_on_right_jury_m3l, right_jury_point_on_right_strut_m3l)
    right_strut_right_jury_connection = m3l.norm(right_strut_point_on_right_jury_m3l - right_jury_point_on_right_strut_m3l)

    # endregion

    # tbw_param_solver.declare_state('wing_span_stretch_coefficients', wing_span_stretch_coefficients, penalty_factor=10) # first output 
    tbw_param_solver.declare_state('wing_span_stretch_coefficients', wing_span_stretch_coefficients, penalty_factor=100)
#     tbw_param_solver.declare_state('wing_span_stretch_coefficients', wing_span_stretch_coefficients)
    tbw_param_solver.declare_state('wing_chord_stretch_coefficients', wing_chord_stretch_coefficients)
    # tbw_param_solver.declare_state('wing_chord_stretch_coefficients', wing_chord_stretch_coefficients, penalty_factor=1000)
    # tbw_param_solver.declare_state('left_strut_span_stretch_coefficients', left_strut_span_stretch_coefficients, penalty_factor=1000)
    tbw_param_solver.declare_state('left_strut_span_stretch_coefficients', left_strut_span_stretch_coefficients)
    # tbw_param_solver.declare_state('left_strut_chord_stretch_coefficients', left_strut_chord_stretch_coefficients)
    tbw_param_solver.declare_state('left_strut_translation_x_coefficients', left_strut_translation_x_coefficients)
    tbw_param_solver.declare_state('left_strut_translation_z_coefficients', left_strut_translation_z_coefficients)
    # tbw_param_solver.declare_state('right_strut_span_stretch_coefficients', right_strut_span_stretch_coefficients, penalty_factor=1000)
    tbw_param_solver.declare_state('right_strut_span_stretch_coefficients', right_strut_span_stretch_coefficients)
    # tbw_param_solver.declare_state('right_strut_chord_stretch_coefficients', right_strut_chord_stretch_coefficients)
    tbw_param_solver.declare_state('right_strut_translation_x_coefficients', right_strut_translation_x_coefficients)
    tbw_param_solver.declare_state('right_strut_translation_z_coefficients', right_strut_translation_z_coefficients)

    tbw_param_solver.declare_state('left_jury_translation_x_coefficients', left_jury_translation_x_coefficients)
    tbw_param_solver.declare_state('left_jury_translation_y_coefficients', left_jury_translation_y_coefficients)
    tbw_param_solver.declare_state('left_jury_stretch_z_coefficients', left_jury_stretch_z_coefficients)
    tbw_param_solver.declare_state('right_jury_translation_x_coefficients', right_jury_translation_x_coefficients)
    tbw_param_solver.declare_state('right_jury_translation_y_coefficients', right_jury_translation_y_coefficients)
    tbw_param_solver.declare_state('right_jury_stretch_z_coefficients', right_jury_stretch_z_coefficients)

    tbw_param_solver.declare_state('gearpod_translation_y_coefficients', gearpod_translation_y_coefficients)
    tbw_param_solver.declare_state('gearpod_translation_x_coefficients', gearpod_translation_x_coefficients)
    tbw_param_solver.declare_state('gearpod_translation_z_coefficients', gearpod_translation_z_coefficients)


    tbw_param_solver.declare_input(name=wing_span_dv.name, input=wing_span)
    tbw_param_solver.declare_input(name=wing_tip_chord_left_dv.name, input=tip_chord_left)
#     tbw_param_solver.declare_input(name=wing_tip_chord_right_dv.name, input=tip_chord_right)
    tbw_param_solver.declare_input(name=wing_root_chord_dv.name, input=root_chord)
    tbw_param_solver.declare_input(name=wing_mid_chord_left_dv.name, input=midpoint_wing_left)
#     tbw_param_solver.declare_input(name=wing_mid_chord_right_dv.name, input=midpoint_wing_right)
    # tbw_param_solver.declare_input(name=left_strut_span_dv.name, input=left_strut_span)
    # tbw_param_solver.declare_input(name='left_strut_chord_dv', input=left_strut_chord)
    # tbw_param_solver.declare_input(name=right_strut_span_dv.name, input=right_strut_span)
    # tbw_param_solver.declare_input(name='right_strut_chord_dv', input=right_strut_chord)
    # tbw_param_solver.declare_input(name='left_jury_height_dv', input=left_jury_height)
    # tbw_param_solver.declare_input(name='right_jury_height_dv', input=right_jury_height)

    tbw_param_solver.declare_input(name='wing_fuselage_connection', input=wing_fuselage_connection)
#     tbw_param_solver.declare_input(name='wing_fuselage_connection', input=wing_fuselage_connection_m3l)
    # tbw_param_solver.declare_input(name='gearpod_left_strut_connection', input=gearpod_left_strut_connection_m3l)
    # tbw_param_solver.declare_input(name='gearpod_right_strut_connection', input=gearpod_right_strut_connection_m3l)
    tbw_param_solver.declare_input(name='gearpod_left_strut_connection', input=gearpod_left_strut_connection)
    tbw_param_solver.declare_input(name='gearpod_right_strut_connection', input=gearpod_right_strut_connection)
    tbw_param_solver.declare_input(name='wing_left_strut_connection', input=wing_left_strut_connection_m3l)
    tbw_param_solver.declare_input(name='wing_right_strut_connection', input=wing_right_strut_connection_m3l)
    # tbw_param_solver.declare_input(name='wing_left_jury_connection', input=wing_left_jury_connection_m3l)
    # tbw_param_solver.declare_input(name='wing_right_jury_connection', input=wing_right_jury_connection_m3l)
    tbw_param_solver.declare_input(name='left_strut_left_jury_connection', input=left_strut_left_jury_connection_m3l)
    tbw_param_solver.declare_input(name='right_strut_right_jury_connection', input=right_strut_right_jury_connection_m3l)

    wing_fuselage_connection_input = m3l.Variable(name='wing_fuselage_connection', shape=(1, ), value=wing_fuselage_connection.value)
    # gearpod_left_strut_connection_input = m3l.Variable(name='gearpod_left_strut_connection', shape=(1, ), value=gearpod_left_strut_connection.value)
    # gearpod_right_strut_connection_input = m3l.Variable(name='gearpod_right_strut_connection', shape=(1, ), value=gearpod_right_strut_connection.value)
    # gearpod_left_strut_connection_input = m3l.Variable(name='gearpod_left_strut_connection', shape=(1, ), value=0.)
    # gearpod_right_strut_connection_input = m3l.Variable(name='gearpod_right_strut_connection', shape=(1, ), value=0.)
#     # wing_left_strut_connection_input = m3l.Variable(name='wing_left_strut_connection', shape=(1, ), value=wing_left_strut_connection.value)
#     # wing_left_jury_connection_input = m3l.Variable(name='wing_left_jury_connection', shape=(1, ), value=wing_left_jury_connection_m3l.value)
#     # wing_right_jury_connection_input = m3l.Variable(name='wing_right_jury_connection', shape=(1, ), value=wing_right_jury_connection_m3l.value)
#     # left_strut_left_jury_connection_input = m3l.Variable(name='left_strut_left_jury_connection', shape=(1, ), value=left_strut_left_jury_connection_m3l.value)
#     # right_strut_right_jury_connection_input = m3l.Variable(name='right_strut_right_jury_connection', shape=(1, ), value=right_strut_right_jury_connection_m3l.value)

#     wing_fuselage_connection_input = m3l.Variable(name='wing_fuselage_connection', shape=(3, ), value=wing_fuselage_connection_m3l.value)
    # gearpod_left_strut_connection_input = m3l.Variable(name='gearpod_left_strut_connection', shape=(3, ), value=gearpod_left_strut_connection_m3l.value)
    # gearpod_right_strut_connection_input = m3l.Variable(name='gearpod_right_strut_connection', shape=(3, ), value=gearpod_right_strut_connection_m3l.value)
# #     gearpod_left_strut_connection_input = m3l.Variable(name='gearpod_left_strut_connection', shape=(3, ), value=np.zeros((3,)))
# #     gearpod_right_strut_connection_input = m3l.Variable(name='gearpod_right_strut_connection', shape=(3, ), value=np.zeros((3,)))
#     wing_left_strut_connection_input = m3l.Variable(name='wing_left_strut_connection', shape=(3, ), value=wing_left_strut_connection_m3l.value)
#     wing_right_strut_connection_input = m3l.Variable(name='wing_right_strut_connection', shape=(3, ), value=wing_right_strut_connection_m3l.value)
#     wing_left_jury_connection_input = m3l.Variable(name='wing_left_jury_connection', shape=(3, ), value=wing_left_jury_connection_m3l.value)
#     wing_right_jury_connection_input = m3l.Variable(name='wing_right_jury_connection', shape=(3, ), value=wing_right_jury_connection_m3l.value)
#     left_strut_left_jury_connection_input = m3l.Variable(name='left_strut_left_jury_connection', shape=(3, ), value=left_strut_left_jury_connection_m3l.value)
#     right_strut_right_jury_connection_input = m3l.Variable(name='right_strut_right_jury_connection', shape=(3, ), value=right_strut_right_jury_connection_m3l.value)

#     wing_fuselage_connection_input = system_model.create_input(name='wing_fuselage_connection', shape=(1, ), value=wing_fuselage_connection.value)
    gearpod_left_strut_connection_input = system_model.create_input(name='gearpod_left_strut_connection', shape=(1, ), val=gearpod_left_strut_connection.value)
    gearpod_right_strut_connection_input = system_model.create_input(name='gearpod_right_strut_connection', shape=(1, ), val=gearpod_right_strut_connection.value)
    # gearpod_right_strut_connection_input = system_model.create_input(name='gearpod_right_strut_connection', shape=(1, ), val=gearpod_right_strut_connection.value)
    # gearpod_left_strut_connection_input = system_model.create_input(name='gearpod_left_strut_connection', shape=(1, ), val=0.)
    # gearpod_right_strut_connection_input = system_model.create_input(name='gearpod_right_strut_connection', shape=(1, ), val=0.)
    # wing_left_strut_connection_input = system_model.create_input(name='wing_left_strut_connection', shape=(1, ), value=wing_left_strut_connection.value)
    # wing_left_jury_connection_input = system_model.create_input(name='wing_left_jury_connection', shape=(1, ), value=wing_left_jury_connection_m3l.value)
    # wing_right_jury_connection_input = system_model.create_input(name='wing_right_jury_connection', shape=(1, ), value=wing_right_jury_connection_m3l.value)
    # left_strut_left_jury_connection_input = system_model.create_input(name='left_strut_left_jury_connection', shape=(1, ), value=left_strut_left_jury_connection_m3l.value)
    # right_strut_right_jury_connection_input = system_model.create_input(name='right_strut_right_jury_connection', shape=(1, ), value=right_strut_right_jury_connection_m3l.value)

#     wing_fuselage_connection_input = system_model.create_input(name='wing_fuselage_connection', shape=(3, ), val=wing_fuselage_connection_m3l.value)
    # gearpod_left_strut_connection_input = system_model.create_input(name='gearpod_left_strut_connection', shape=(3, ), val=gearpod_left_strut_connection_m3l.value)
    # gearpod_right_strut_connection_input = system_model.create_input(name='gearpod_right_strut_connection', shape=(3, ), val=gearpod_right_strut_connection_m3l.value)
    # gearpod_left_strut_connection_input = system_model.create_input(name='gearpod_left_strut_connection', shape=(3, ), value=np.zeros((3,)))
    # gearpod_right_strut_connection_input = system_model.create_input(name='gearpod_right_strut_connection', shape=(3, ), value=np.zeros((3,)))
    wing_left_strut_connection_input = system_model.create_input(name='wing_left_strut_connection', shape=(3, ), val=wing_left_strut_connection_m3l.value)
    wing_right_strut_connection_input = system_model.create_input(name='wing_right_strut_connection', shape=(3, ), val=wing_right_strut_connection_m3l.value)
    # wing_left_jury_connection_input = system_model.create_input(name='wing_left_jury_connection', shape=(3, ), val=wing_left_jury_connection_m3l.value)
    # wing_right_jury_connection_input = system_model.create_input(name='wing_right_jury_connection', shape=(3, ), val=wing_right_jury_connection_m3l.value)
    left_strut_left_jury_connection_input = system_model.create_input(name='left_strut_left_jury_connection', shape=(3, ), val=left_strut_left_jury_connection_m3l.value)
    right_strut_right_jury_connection_input = system_model.create_input(name='right_strut_right_jury_connection', shape=(3, ), val=right_strut_right_jury_connection_m3l.value)


    parameterization_inputs = {
        wing_span_dv.name : wing_span_dv,
        wing_tip_chord_left_dv.name : wing_tip_chord_left_dv,
        # wing_tip_chord_right_dv.name : wing_tip_chord_right_dv,
        wing_root_chord_dv.name : wing_root_chord_dv,
        wing_mid_chord_left_dv.name : wing_mid_chord_left_dv,
        # wing_mid_chord_right_dv.name : wing_mid_chord_right_dv,
        # left_strut_span_dv.name : left_strut_span_dv,
        # 'left_strut_chord_dv' : left_strut_chord_dv,
        # right_strut_span_dv.name : right_strut_span_dv,
        # 'right_strut_chord_dv' : right_strut_chord_dv,  
        # 'left_jury_height_dv' : left_jury_height_dv,
        # 'right_jury_height_dv' : right_jury_height_dv,
        'wing_fuselage_connection' : wing_fuselage_connection_input,
        'gearpod_left_strut_connection' : gearpod_left_strut_connection_input,
        'gearpod_right_strut_connection' : gearpod_right_strut_connection_input,
        'wing_left_strut_connection' : wing_left_strut_connection_input,
        'wing_right_strut_connection' : wing_right_strut_connection_input,
        # 'wing_left_jury_connection' : wing_left_jury_connection_input,
        # 'wing_right_jury_connection' : wing_right_jury_connection_input,
        'left_strut_left_jury_connection' : left_strut_left_jury_connection_input,
        'right_strut_right_jury_connection' : right_strut_right_jury_connection_input,
    }

    # Evaluate parameterization solver
    outputs_dict = tbw_param_solver.evaluate(inputs=parameterization_inputs)

    # RE-ASSIGNING COEFFICIENTS AFTER INNER OPTIMIZATION
 
    coefficients_list = []
    b_spline_names_list = []

    # region Wing
    wing_span_stretch_coefficients = outputs_dict['wing_span_stretch_coefficients']
    wing_span_stretch_coefficients = outputs_dict['wing_span_stretch_coefficients']
    wingspan_stretch_b_spline = bsp.BSpline(name='wingspan_stretch_b_spline', space=linear_b_spline_curve_2_dof_space,
                                                        coefficients=wing_span_stretch_coefficients, num_physical_dimensions=1)

    wing_chord_stretch_coefficients = outputs_dict['wing_chord_stretch_coefficients']
    wing_chord_stretch_b_spline = bsp.BSpline(name='wing_chord_stretch_b_spline', space=cubic_b_spline_curve_5_dof_space,
                                                    coefficients=wing_chord_stretch_coefficients, num_physical_dimensions=1)

    wing_ffd_block_sect_param.add_sectional_rotation(name='wing_twist', axis=1)
    geometry_dv = True
    # geometry_dv = False

    wing_twist_coefficients = system_model.create_input(name='wing_twist_coefficients', shape= (3,), 
                                                        val=np.array([0., 0., 0.]), dv_flag = geometry_dv, 
                                                        lower = np.array([-10., 0., -10.]), 
                                                        upper = np.array([10., 0., 10.]))

    wing_twist_b_spline = bsp.BSpline(name='wing_twist_b_spline', space=linear_b_spline_curve_3_dof_space,
                                                coefficients=wing_twist_coefficients, num_physical_dimensions=1)

    defg = wing_twist_coefficients[np.array([2])]- wing_twist_coefficients[np.array([1])] - wing_twist_coefficients[np.array([0])]
    system_model.register_output(defg)
    system_model.add_constraint(defg, equals=0)

    section_parametric_coordinates = np.linspace(0., 1., wing_ffd_block_sect_param.num_sections).reshape((-1,1))
    wing_wingspan_stretch = wingspan_stretch_b_spline.evaluate(section_parametric_coordinates)
    wing_sectional_chord_stretch = wing_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
    wing_sectional_twist = wing_twist_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'wing_span_stretch': wing_wingspan_stretch,
            'wing_chord_stretch': wing_sectional_chord_stretch,
            'wing_twist': wing_sectional_twist,
                                }
    
    wing_ffd_block_coefficients = wing_ffd_block_sect_param.evaluate(sectional_parameters, plot=wing_plot)
    wing_coefficients = wing_ffd_block.evaluate(wing_ffd_block_coefficients, plot=wing_plot)
    b_spline_names_list.append(wing.b_spline_names)
    # endregion

    # region strut 

    # region left strut try
    left_strut_translation_x_coefficients = outputs_dict['left_strut_translation_x_coefficients']
    left_strut_translation_z_coefficients = outputs_dict['left_strut_translation_z_coefficients']

    left_strut_span_stretch_coefficients = outputs_dict['left_strut_span_stretch_coefficients']
    left_strut_span_strech_b_spline = bsp.BSpline(name='left_strut_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=left_strut_span_stretch_coefficients, num_physical_dimensions=1)
    
    left_strut_ffd_block_translation_x_b_spline = bsp.BSpline(name='left_strut_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_strut_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    left_strut_ffd_block_translation_z_b_spline = bsp.BSpline(name='left_strut_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=left_strut_translation_z_coefficients, 
                                                        num_physical_dimensions=1)
    
#     # addtional part included now
#     left_strut_ffd_block_sect_param.add_sectional_translation(name='left_strut_sweep', axis=0)

#     left_strut_sweep_coefficients = system_model.create_input(name='left_strut_sweep_coefficients', shape=(2,),
#                                                 val=np.array([0., 1.6]))    
#     left_strut_sweep_b_spline = bsp.BSpline(name='left_strut_sweep_b_spline', space=linear_b_spline_curve_2_dof_space,
#                                                 coefficients=left_strut_sweep_coefficients, num_physical_dimensions=1)
    
#     # additional part ends

    section_parametric_coordinates = np.linspace(0., 1., left_strut_ffd_block_sect_param.num_sections).reshape((-1,1))
    left_strut_translation_x = left_strut_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    left_strut_translation_z = left_strut_ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)
    left_strut_span_stretch = left_strut_span_strech_b_spline.evaluate(section_parametric_coordinates)
#     left_strut_sweep = left_strut_sweep_b_spline.evaluate(section_parametric_coordinates)
    
    sectional_parameters = {
            'left_strut_translation_x': left_strut_translation_x,
            'left_strut_translation_z': left_strut_translation_z,
            'left_strut_span_stretch': left_strut_span_stretch,
        #     'left_strut_sweep': left_strut_sweep,
    }

    left_strut_ffd_block_coefficients = left_strut_ffd_block_sect_param.evaluate(sectional_parameters, plot=left_strut_plot)
    left_strut_coefficients = left_strut_ffd_block.evaluate(left_strut_ffd_block_coefficients, plot=left_strut_plot)
    b_spline_names_list.append(strut_left.b_spline_names)

    # endregion

    # region right strut new 
    right_strut_translation_x_coefficients = outputs_dict['right_strut_translation_x_coefficients']
    right_strut_translation_z_coefficients = outputs_dict['right_strut_translation_z_coefficients']

    right_strut_span_stretch_coefficients = outputs_dict['right_strut_span_stretch_coefficients']
    right_strut_span_strech_b_spline = bsp.BSpline(name='right_strut_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=right_strut_span_stretch_coefficients, num_physical_dimensions=1)
    
    right_strut_ffd_block_translation_x_b_spline = bsp.BSpline(name='right_strut_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_strut_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    right_strut_ffd_block_translation_z_b_spline = bsp.BSpline(name='right_strut_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=right_strut_translation_z_coefficients, 
                                                        num_physical_dimensions=1)
    
#     # addtional part included now
#     right_strut_ffd_block_sect_param.add_sectional_translation(name='right_strut_sweep', axis=0)

#     right_strut_sweep_coefficients = system_model.create_input(name='right_strut_sweep_coefficients', shape=(2,),
#                                                 val=np.array([1.6, 0.]))    
#     right_strut_sweep_b_spline = bsp.BSpline(name='right_strut_sweep_b_spline', space=linear_b_spline_curve_2_dof_space,
#                                                 coefficients=right_strut_sweep_coefficients, num_physical_dimensions=1)
    
#     # additional part ends
        
    section_parametric_coordinates = np.linspace(0., 1., right_strut_ffd_block_sect_param.num_sections).reshape((-1,1))
    right_strut_translation_x = right_strut_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    right_strut_translation_z = right_strut_ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)
    right_strut_span_stretch = right_strut_span_strech_b_spline.evaluate(section_parametric_coordinates)
#     right_strut_sweep = right_strut_sweep_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'right_strut_translation_x': right_strut_translation_x,
            'right_strut_translation_z': right_strut_translation_z,
            'right_strut_span_stretch': right_strut_span_stretch,
        #     'right_strut_sweep': right_strut_sweep,
    }

    right_strut_ffd_block_coefficients = right_strut_ffd_block_sect_param.evaluate(sectional_parameters, plot=right_strut_plot)
    right_strut_coefficients = right_strut_ffd_block.evaluate(right_strut_ffd_block_coefficients, plot=right_strut_plot)
    b_spline_names_list.append(strut_right.b_spline_names)

    # endregion

    # endregion

    # region Jury

    # region left Jury 
    left_jury_translation_x_coefficients = outputs_dict['left_jury_translation_x_coefficients']
    left_jury_translation_y_coefficients = outputs_dict['left_jury_translation_y_coefficients']
    left_jury_stretch_z_coefficients = outputs_dict['left_jury_stretch_z_coefficients']

    left_jury_ffd_block_translation_x_b_spline = bsp.BSpline(name='left_jury_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                             coefficients=left_jury_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    left_jury_ffd_block_translation_y_b_spline = bsp.BSpline(name='left_jury_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                             coefficients=left_jury_translation_y_coefficients, 
                                                        num_physical_dimensions=1)

    left_jury_ffd_block_stretch_z_b_spline = bsp.BSpline(name='left_jury_stretch_z_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                         coefficients=left_jury_stretch_z_coefficients, 
                                                        num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., left_jury_ffd_block_sect_param.num_sections).reshape((-1,1))
    left_jury_translation_x = left_jury_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    left_jury_translation_y = left_jury_ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
    left_jury_stretch_z = left_jury_ffd_block_stretch_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'left_jury_translation_x': left_jury_translation_x,
            'left_jury_translation_y': left_jury_translation_y,
            'left_jury_stretch_z': left_jury_stretch_z,
    }
    
    left_jury_ffd_block_coefficients = left_jury_ffd_block_sect_param.evaluate(sectional_parameters, plot=left_jury_plot)
    left_jury_coefficients = left_jury_ffd_block.evaluate(left_jury_ffd_block_coefficients, plot=left_jury_plot)
    b_spline_names_list.append(jury_left.b_spline_names)

    # endregion

    # region right Jury
    right_jury_translation_x_coefficients = outputs_dict['right_jury_translation_x_coefficients']
    right_jury_translation_y_coefficients = outputs_dict['right_jury_translation_y_coefficients']
    right_jury_stretch_z_coefficients = outputs_dict['right_jury_stretch_z_coefficients']

    right_jury_ffd_block_translation_x_b_spline = bsp.BSpline(name='right_jury_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                              coefficients=right_jury_translation_x_coefficients, num_physical_dimensions=1)
        
    right_jury_ffd_block_translation_y_b_spline = bsp.BSpline(name='right_jury_translation_y_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                              coefficients=right_jury_translation_y_coefficients, num_physical_dimensions=1)
        
    right_jury_ffd_block_stretch_z_b_spline = bsp.BSpline(name='right_jury_stretch_z_bspline', space=constant_b_spline_curve_1_dof_space, 
                                                          coefficients=right_jury_stretch_z_coefficients, num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., right_jury_ffd_block_sect_param.num_sections).reshape((-1,1))
    right_jury_translation_x = right_jury_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    right_jury_translation_y = right_jury_ffd_block_translation_y_b_spline.evaluate(section_parametric_coordinates)
    right_jury_stretch_z = right_jury_ffd_block_stretch_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'right_jury_translation_x': right_jury_translation_x,
            'right_jury_translation_y': right_jury_translation_y,
            'right_jury_stretch_z': right_jury_stretch_z,
    }
    
    right_jury_ffd_block_coefficients = right_jury_ffd_block_sect_param.evaluate(sectional_parameters, plot=right_jury_plot)
    right_jury_coefficients = right_jury_ffd_block.evaluate(right_jury_ffd_block_coefficients, plot=right_jury_plot)
    b_spline_names_list.append(jury_right.b_spline_names)
    
    # endregion

    # endregion

    # region gearpod

    gearpod_span_stretch_coefficients = outputs_dict['gearpod_translation_y_coefficients']

    gearpod_span_strech_b_spline = bsp.BSpline(name='gearpod_span_b_spline', space=linear_b_spline_curve_2_dof_space, 
                                            coefficients=gearpod_span_stretch_coefficients, num_physical_dimensions=1)
        
       
    gearpod_translation_x_coefficients = outputs_dict['gearpod_translation_x_coefficients']
    gearpod_translation_z_coefficients = outputs_dict['gearpod_translation_z_coefficients']
    
    gearpod_ffd_block_translation_x_b_spline = bsp.BSpline(name='gearpod_translation_x_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=gearpod_translation_x_coefficients, 
                                                        num_physical_dimensions=1)
        
    gearpod_ffd_block_translation_z_b_spline = bsp.BSpline(name='gearpod_translation_z_bspline', space=constant_b_spline_curve_1_dof_space, coefficients=gearpod_translation_z_coefficients, 
                                                        num_physical_dimensions=1)

    section_parametric_coordinates = np.linspace(0., 1., gearpod_ffd_block_sect_param.num_sections).reshape((-1,1))
    gearpod_translation_y = gearpod_span_strech_b_spline.evaluate(section_parametric_coordinates)
#     gearpod_chord_stretch = gearpod_chord_stretch_b_spline.evaluate(section_parametric_coordinates)
    gearpod_translation_x = gearpod_ffd_block_translation_x_b_spline.evaluate(section_parametric_coordinates)
    gearpod_translation_z = gearpod_ffd_block_translation_z_b_spline.evaluate(section_parametric_coordinates)

    sectional_parameters = {
            'gearpod_translation_y': gearpod_translation_y,
        #     'gearpod_chord_stretch': gearpod_chord_stretch,
        'gearpod_translation_x' : gearpod_translation_x,
        'gearpod_translation_z' : gearpod_translation_z,
    }
    # gearpod_plot = True
    gearpod_ffd_block_coefficients = gearpod_ffd_block_sect_param.evaluate(sectional_parameters, plot=gearpod_plot)
    gearpod_coefficients = gearpod_ffd_block.evaluate(gearpod_ffd_block_coefficients, plot=gearpod_plot)
    b_spline_names_list.append(gearpod.b_spline_names)

    # endregion

    coefficients_list.append(wing_coefficients)
    coefficients_list.append(left_strut_coefficients)
    coefficients_list.append(right_strut_coefficients)
    coefficients_list.append(left_jury_coefficients)
    coefficients_list.append(right_jury_coefficients)
    coefficients_list.append(gearpod_coefficients)

    geometry.assign_coefficients(coefficients=coefficients_list, b_spline_names=b_spline_names_list)

    # geometry_flag = True
    if geometry_flag:
        geometry.plot()

# region Update all the meshes

# wing mesh 
wing_meshes = wing_meshes.update(geometry=geometry)
# left strut mesh 
left_strut_meshes = left_strut_meshes.update(geometry=geometry)
# right strut mesh 
right_strut_meshes = right_strut_meshes.update(geometry=geometry)
# left jury mesh 
left_jury_meshes = left_jury_meshes.update(geometry=geometry)
# right jury mesh 
right_jury_meshes = right_jury_meshes.update(geometry=geometry)

# # Component surface areas
# component_list = [wing,strut,htail]
# surface_area_list = compute_component_surface_area(
#     component_list=component_list,
#     geometry=geometry,
#     parametric_mesh_grid_num=20,
#     plot=False,
# )

# Component surface areas
component_list = [Fuselage, wing, htail, vtail, jury, gearpod, strut]
surface_area_list = compute_component_surface_area(
    component_list=component_list,
    geometry=geometry,
    parametric_mesh_grid_num=20,
    plot=False,
)

S_ref_area = surface_area_list[1] / 1.969576502
h_tail_area = surface_area_list[2] / 2.1
v_tail_area = surface_area_list[3] / 2.18
jury_area = surface_area_list[4] / 2.11
strut_area = surface_area_list[6] / 2.07
wing_AR = ((wing_span_dv)**2 )/ S_ref_area
print('wing area', S_ref_area.value)
print('htail area', h_tail_area.value)
print('v_tail area', v_tail_area.value)
print('jury area', jury_area.value)
print('strut area', strut_area.value)
print('wing_AR', wing_AR.value)