import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.tools import units
from aerosandbox.library.propulsion_electric import motor_electric_performance
from aerosandbox.library.aerodynamics.components import CDA_control_surface_gaps, CDA_control_linkage
import pandas as pd
import os
import csv
from alive_progress import alive_bar, animations

custom = animations.bar_factory(chars='==', tip='☁️✈️', background='⠈⠐⠠⢀⡀⠄⠂⠁', borders = ('🛫→', '←🛬'), errors='💥')

def opt(params, mission=None):
    opti = asb.Opti()

    #initial constants & assumptions
    """Assumptions:
    CHECK DISPLACEMENTS
    1. Assuming conventional tail
    2. Assuming 2 fuel pods
    3. Assuming 1 motor
    4. fuel pods at b/5

    TODO:
        sandwich spar 
            strength
            stiffness
        sandwich tail
            strength
            stiffness
        rectangular boom 

    """
    """
    0.04

    CDCL for sd7032

    CDCL
    #CL1  CD1  CL2  CD2  CL3  CD3
    -0.3  0.01 0.6  0.005 1.6 0.018
    """

    """
    CDCL for naca0010

    CDCL
    #CL1  CD1  CL2  CD2  CL3  CD3
    -1.5  0.026 0  0.045 1.5 0.026
    """

    airfoil = asb.Airfoil("sd7032")
    # airfoil = asb.Airfoil("rg12a")
    symmetric_airfoil = asb.Airfoil("naca0010")

    g= 9.81
    rho=1.225
    downwind_length = 1000*units.foot



    #### WING SIZING
    b = 6*units.foot ### doing this for wing weight model, I want to use int(b/spacing_between_ribs) as a constant  
    def twist_func(x, twist):
        return -twist/(b/2)*x
    def taper_func(x, rc, taper):
        return (taper*rc-rc)/b*x + rc
    def constant_quarter_chord(root_chord, tip_chord):  
        return root_chord/4 - tip_chord/4

    """---- DENSITIES AND MATERIAL PROPERTIES"""
    carbon_fiber_density = 1.75*1000 #kg/m^3
    carbon_fiber_yield_strength = 600e6 #Pa
    carbon_fiber_youngs_modulus = 230e9 #Pa
    carbon_fiber_shear_modulus = 50e9 #Pa
    carbon_fiber_layup_epoxy_factor = 2.2

    fiberglass_density = 2.6*1000 #kg/m^3
    fiberglass_youngs_modulus = 70e9 #Pa
    fiberglass_shear_modulus = 30e9 #Pa
    fiberglass_layup_epoxy_factor = 2.2

    basswood_density = 320 #kg/m^3
    basswood_youngs_modulus = 10e9 #Pa
    basswood_shear_modulus = 3.8e9 #Pa
    basswood_layup_epoxy_factor = 1.2
    basswood_shear_strength = 6.8e6 #Pa

    plywood_density = 550 #kg/m^3

    balsa_density = 160 #kg/m^3
    balsa_youngs_modulus = 4e9 #Pa
    balsa_shear_modulus = 1.5e9 #Pa
    balsa_layup_epoxy_factor = 1.2
    balsa_shear_strength = 2.1e6 #Pa

    foam_density = 48 #kg/m^3




    #Main wing
    #untapered
    # rc = opti.variable(init_guess=0.49, lower_bound=0.25, upper_bound=1.0, scale=0.1, category="rc")  # Root chord
    # tc = rc
    # wing = asb.Wing(name='Wing',
    #                 symmetric=True,
    #                 xsecs = [
    #                     asb.WingXSec(
    #                         xyz_le=[0, 0, 0],
    #                         chord=rc,
    #                         twist=0,
    #                         airfoil=airfoil,
    #                         control_surfaces=[asb.ControlSurface(name="Flap", hinge_point=0.6, deflection=0)],
    #                     ),
    #                     asb.WingXSec(
    #                         xyz_le=[0, b/4, 0],
    #                         chord=rc,
    #                         twist=0,
    #                         airfoil=airfoil,
    #                         control_surfaces=[asb.ControlSurface(name="Aileron", symmetric=False, hinge_point=0.75, deflection=0)],
    #                     ),
    #                     asb.WingXSec(
    #                         xyz_le=[0, b/2, 0],
    #                         chord=rc,
    #                         twist=0,
    #                         airfoil=airfoil,
    #                     ),
    #                 ])

    #half tapered

    # rc = opti.variable(init_guess=0.8, lower_bound=0.25, upper_bound=1.0, scale=0.1, category="rc")  # Root chord
    # tc = opti.variable(init_guess=0.1, lower_bound=0.1, upper_bound=0.5, scale=0.1, category="tc")  # Taper ratio
    # wing = asb.Wing(name='Wing',
    #                 symmetric=True,
    #                 xsecs = [
    #                     asb.WingXSec(
    #                         xyz_le=[0, 0, 0],
    #                         chord=rc,
    #                         twist=0,
    #                         airfoil=airfoil,
    #                         control_surfaces=[asb.ControlSurface(name="Flap", hinge_point=0.6, deflection=0)],
    #                     ),
    #                     asb.WingXSec(
    #                         xyz_le=[0, b/4, 0],
    #                         chord=rc,
    #                         twist=0,
    #                         airfoil=airfoil,
    #                         control_surfaces=[asb.ControlSurface(name="Aileron", symmetric=False, hinge_point=0.75, deflection=0)],
    #                     ),
    #                     asb.WingXSec(
    #                         xyz_le=[constant_quarter_chord(rc, tc), b/2, 0],
    #                         chord=tc,
    #                         twist=0,
    #                         airfoil=airfoil,
    #                     ),
    #                 ])

    #fully tapered
    def fully_tapered(rc, tc):
        return (rc+tc)/2
    rc = opti.variable(init_guess=0.8, lower_bound=0.25, upper_bound=1.0, scale=0.1, category="rc")  # Root chord
    tc = opti.variable(init_guess=0.1, lower_bound=0.1, upper_bound=1.0, scale=0.1, category="tc")  # Taper ratio
    taper = tc/rc
    washout = 5
    dihedral_angle = 2
    # opti.subject_to(taper == 1.0)

    wing_secs = [0, 0.04, 0.43, 1.82/2]
    control_surface_for_section = [[], [asb.ControlSurface(name="Flap", hinge_point=0.6, deflection=0)], [asb.ControlSurface(name="Aileron", symmetric=False, hinge_point=0.75, deflection=0)], []]
    def taper_func(y, rc, taper):
        return (taper*rc-rc)/(b/2)*y + rc

    def dihedral(y, dihedral):
        return y*np.tan(np.radians(dihedral))

    def current_sec(i):
        y = wing_secs[i]
        chord = taper_func(y, rc, taper)
        x_disp = constant_quarter_chord(rc, chord)
        z_disp = dihedral(y, dihedral_angle)
        twist = twist_func(y, washout)
        control_surfaces = control_surface_for_section[i]
        return asb.WingXSec(xyz_le=[x_disp, y, z_disp], chord=chord, twist=twist, airfoil=airfoil, control_surfaces=control_surfaces)

    wing = asb.Wing(name='Wing',
                    symmetric=True,
                    xsecs = [current_sec(i) for i in range(len(wing_secs))])

    AR = wing.aspect_ratio()
    S = wing.area() 
    c_bar = wing.mean_geometric_chord()
    # sweep = 
    tail_dis = opti.variable(init_guess=1.25, lower_bound=0.9, upper_bound=2.5, scale=0.1, category="tail_dis", freeze=False)  # Distance from wing leading edge to horizontal tail

    opti.subject_to([
        # taper == params['taper'],
        # S == params['S'],
        ])

    ##### VERTICAL TAIL SIZING
    v_b = opti.variable(init_guess=0.4, lower_bound=0.1, upper_bound=1.8, scale=0.1, category="v_b", freeze=False)  # Span
    v_c = opti.variable(init_guess=0.24, lower_bound=0.15, upper_bound=1, scale=0.1, category="v_c", freeze=False)  # Chord
    vertical_tail = asb.Wing(name='Vertical Tail',
                            xsecs = [
                                asb.WingXSec(
                                    xyz_le=[0, 0, 0],
                                    chord=v_c,
                                    twist=0,
                                    airfoil=symmetric_airfoil,
                                    control_surfaces=[asb.ControlSurface(name="Rudder", hinge_point=0.75, deflection=0)],
                                ),
                                asb.WingXSec(
                                    xyz_le=[0, 0, v_b],
                                    chord=v_c,
                                    twist=0,
                                    airfoil=symmetric_airfoil,
                                ),
                            ],
    ).translate([tail_dis, 0, 0])
    Sv = vertical_tail.area(type='xz')




    ##### HORITZONTAL TAIL SIZING
    h_b = opti.variable(init_guess=0.8, lower_bound=0.1, upper_bound=1.8, scale=0.1, category="h_b", freeze=False)  # Span
    h_c = opti.variable(init_guess=0.24, lower_bound=0.15, upper_bound=1, scale=0.1, category="h_c", freeze=False)  # Chord


    horizontal_tail = asb.Wing(name='Horizontal Tail',
                            symmetric=True,
                            xsecs = [
                                asb.WingXSec(
                                    xyz_le=[0, 0, 0],
                                    chord=h_c,
                                    twist=0,
                                    airfoil=symmetric_airfoil,
                                    control_surfaces=[asb.ControlSurface(name="Elevator", hinge_point=0.75, deflection=0)],
                                ),
                                asb.WingXSec(
                                    xyz_le=[0, h_b/2, 0],
                                    chord=h_c,
                                    twist=0,
                                    airfoil=symmetric_airfoil,
                                ),
                                ]
    ).translate([tail_dis, 0, 0])
    V_tail_volume = vertical_tail.area()*tail_dis/(S*b)
    H_tail_volume = horizontal_tail.area()*tail_dis/(S*c_bar)

    wings = [wing, horizontal_tail, vertical_tail]
    opti.subject_to([h_c == v_c,
                    # H_tail_volume > 0.3,
                    # H_tail_volume < 0.6,
                    # V_tail_volume > 0.035,
                    # V_tail_volume < 0.05,
                    # h_c == 0.235,
                    # h_b == 0.815,
                    # v_b == 0.42,
                    V_tail_volume == 0.07,
                    H_tail_volume == 0.54,
                    horizontal_tail.aspect_ratio() < 4.0
                    ])
    Sh = horizontal_tail.area()



    #### POD GEOMETRY DEFINITION
    def elliptical_radius(x, length, val):
        x_over_c = x / length
        return val / 2 * (1 - (2 * x_over_c - 1) ** 2) ** 0.5

    diameter = 0.0635 #m
    pod_length = 0.203 #m
    min_x = -0.05
    max_x = min_x + pod_length
    number_of_pods = 2
    z_disp_pod = -0.15
    xs = np.arange(0, pod_length, 0.01)

    #plane
    fuselages = []
    Massprops = asb.MassProperties(mass=0)


    ##### WING MASS DEFINITION
    #ribs
    spacing_between_ribs = 0.15
    rib_thickness = 0.0032 #m
    rib_density = plywood_density # kg/m^3 --plywood density
    rib_area = airfoil.area()/2

    rib_epoxy_factor = 1.2

    wing_mass_per_rib = rib_thickness*rib_area*rib_density*rc*rib_epoxy_factor
    centroidxtc, centroidytc, = airfoil.centroid()
    num_ribs = int(b/spacing_between_ribs) #constant
    wing_rib_mass_props = asb.MassProperties(mass=0)

    #tapered
    for i in range(int(num_ribs/2)):
        chord = taper_func(spacing_between_ribs*i, rc, taper)
        rib_mass = rib_thickness*rib_density*chord*rib_area
        wing_rib_mass_props = wing_rib_mass_props + asb.MassProperties(mass = rib_mass, x_cg = centroidxtc*chord + constant_quarter_chord(rc, taper_func(spacing_between_ribs*i, rc, taper)), y_cg = spacing_between_ribs*i, z_cg = centroidytc*rc) + asb.MassProperties(mass = rib_mass, x_cg = centroidxtc*chord + constant_quarter_chord(rc, taper_func(spacing_between_ribs*i, rc, taper)), y_cg = -spacing_between_ribs*i, z_cg = centroidytc*rc)
    # Massprops = Massprops + wing_rib_mass_props

    wing_mass_props = wing_rib_mass_props
    #monokote
    wing_surface_area = wing.area(type='wetted') #m^2
    tail_surface_area = vertical_tail.area(type="wetted") + horizontal_tail.area(type="wetted") #m^2

    #assume surface material is monokote
    monokote_density = 9/1000/0.092903#kg/m^2
    monokote_mass = (wing_surface_area)*monokote_density
    wing_monokote_mass_props = asb.MassProperties(mass = monokote_mass/2, x_cg = rc/2, y_cg = b/2, z_cg = 0) + asb.MassProperties(mass = monokote_mass/2, x_cg = rc/2, y_cg = -b/2, z_cg = 0)
    wing_mass_props = wing_mass_props + wing_monokote_mass_props

    #carbon leading edge
    carbon_thickness = 0.06/1000 #m (0.06 mm)
    leading_edge_area = airfoil.perimeter() * (rc+tc)/2 / 5 #5 is an estimate for the surface area of just the front of the wing
    mass_leading_edge = carbon_fiber_density*leading_edge_area*carbon_thickness*b/2*carbon_fiber_layup_epoxy_factor
    wing_carbon_leading_edge_mass_props = asb.MassProperties(mass = mass_leading_edge, x_cg = rc/5, y_cg = b/2, z_cg = 0) + asb.MassProperties(mass = mass_leading_edge, x_cg = rc/2, y_cg = -b/2, z_cg = 0)
    wing_mass_props = wing_mass_props + wing_carbon_leading_edge_mass_props

    #control surfaces
    #flap
    # flap_control_surface = control_surface_for_section[1][0]
    flap_mass = 0.075
    flap_mass_props = asb.MassProperties(mass = flap_mass, x_cg = rc/2, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = flap_mass, x_cg = rc/2, y_cg = -b/4, z_cg = 0)

    #aileron
    aileron_mass = 0.053
    aileron_mass_props = asb.MassProperties(mass = aileron_mass, x_cg = rc/2, y_cg = 3*b/4, z_cg = 0) + asb.MassProperties(mass = aileron_mass, x_cg = rc/2, y_cg = -3*b/4, z_cg = 0)

    wing_mass_props = wing_mass_props + flap_mass_props + aileron_mass_props

    Massprops = Massprops + wing_mass_props




    # #### HORIZONTAL TAIL MASS DEFINITION
    # horizontal_tail_mass_props = asb.MassProperties(mass=0)
    # rib_area_tail = symmetric_airfoil.area()*h_c
    # tail_mass_per_rib = rib_thickness*rib_area_tail*rib_density*v_c
    # num_ribs = h_b/spacing_between_ribs
    # ht_rib_mass_props = asb.MassProperties(mass = tail_mass_per_rib*num_ribs, x_cg=tail_dis+v_c/2, y_cg = h_b/4, z_cg = 0) + asb.MassProperties(mass = tail_mass_per_rib*num_ribs, x_cg=tail_dis+v_c/2, y_cg = -h_b/4, z_cg = 0)
    # horizontal_tail_mass_props = horizontal_tail_mass_props + ht_rib_mass_props

    # ht_spardiameter = symmetric_airfoil.max_thickness()*v_c
    # ht_spar_thickness = 0.001
    # ht_spar_density = (np.pi*(ht_spardiameter/2)**2-np.pi*(ht_spardiameter/2 - ht_spar_thickness)**2)*carbon_fiber_density
    # ht_spar_mass = ht_spar_density*v_b
    # ht_spar_mass_props = asb.MassProperties(mass = ht_spar_mass/2, x_cg = tail_dis+v_c/4, y_cg = v_b/2, z_cg = 0) + asb.MassProperties(mass = ht_spar_mass/2, x_cg = tail_dis+v_c/4, y_cg = -v_b/2, z_cg = 0)
    # horizontal_tail_mass_props = horizontal_tail_mass_props + ht_spar_mass_props

    # monokote_mass = (horizontal_tail.area(type="wetted") + vertical_tail.area(type="wetted"))*monokote_density
    # ht_monokote_mass_props = asb.MassProperties(mass = monokote_mass/2, x_cg = tail_dis+v_c/4, y_cg = h_b/4, z_cg = 0) + asb.MassProperties(mass = monokote_mass/2, x_cg = tail_dis+v_c/4, y_cg = -h_b/4, z_cg = 0)
    # horizontal_tail_mass_props = horizontal_tail_mass_props + ht_monokote_mass_props

    # max_thick_tail = symmetric_airfoil.max_thickness()
    # tail_spar_diameter = max_thick_tail*v_c
    # tail_spar_thickness = 0.001 #m = 2mm
    # spar_density_per_length_tail = (np.pi*(tail_spar_diameter/2)**2-np.pi*(tail_spar_diameter/2 - tail_spar_thickness)**2)*carbon_fiber_density
    # spar_mass_tail = spar_density_per_length_tail*h_b
    # ht_spar_mass_props_tail = asb.MassProperties(mass = spar_mass_tail/2, x_cg = tail_dis+v_c/4, y_cg = h_b/4, z_cg = 0) + asb.MassProperties(mass = spar_mass_tail/2, x_cg = tail_dis+v_c/4, y_cg = -h_b/4, z_cg = 0)
    # horizontal_tail_mass_props = horizontal_tail_mass_props + ht_spar_mass_props_tail

    # Massprops = Massprops + horizontal_tail_mass_props






    # #### VERTICAL TAIL MASS DEFINITION
    # vertical_tail_mass_props = asb.MassProperties(mass=0)
    # num_ribs = v_b/spacing_between_ribs
    # vt_rib_mass_props = asb.MassProperties(mass = tail_mass_per_rib*num_ribs, x_cg=tail_dis+v_c/2, y_cg = 0, z_cg = v_b/2)
    # vertical_tail_mass_props = vertical_tail_mass_props + vt_rib_mass_props

    # vt_spar_diameter = symmetric_airfoil.max_thickness()*v_c
    # vt_spar_thickness = 0.001
    # vt_spar_density = (np.pi*(vt_spar_diameter/2)**2-np.pi*(vt_spar_diameter/2 - vt_spar_thickness)**2)*carbon_fiber_density
    # vt_spar_mass = vt_spar_density*v_b
    # vt_spar_mass_props = asb.MassProperties(mass = vt_spar_mass, x_cg = tail_dis+v_c/4, y_cg = 0, z_cg = v_b/2)
    # vertical_tail_mass_props = vertical_tail_mass_props + vt_spar_mass_props

    # monokote_mass = (horizontal_tail.area(type="wetted") + vertical_tail.area(type="wetted"))*monokote_density
    # vt_monokote_mass_props = asb.MassProperties(mass = monokote_mass/2, x_cg = tail_dis+v_c/4, y_cg = h_b/4, z_cg = 0) + asb.MassProperties(mass = monokote_mass/2, x_cg = tail_dis+v_c/4, y_cg = -h_b/4, z_cg = 0)
    # vertical_tail_mass_props = vertical_tail_mass_props + vt_monokote_mass_props

    # max_thick_tail = symmetric_airfoil.max_thickness()
    # spar_density_per_length_tail = (np.pi*(tail_spar_diameter/2)**2-np.pi*(tail_spar_diameter/2 - tail_spar_thickness)**2)*carbon_fiber_density
    # spar_mass_tail = spar_density_per_length_tail*h_b
    # vt_spar_mass_props_tail = asb.MassProperties(mass = spar_mass_tail/2, x_cg = tail_dis+v_c/4, y_cg = h_b/4, z_cg = 0) + asb.MassProperties(mass = spar_mass_tail/2, x_cg = tail_dis+v_c/4, y_cg = -h_b/4, z_cg = 0)
    # vertical_tail_mass_props = vertical_tail_mass_props + vt_spar_mass_props_tail

    # Massprops = Massprops + vertical_tail_mass_props




    #FOAM TAIL MASS ESTIMATE

    ## HORIZONTAL TAIL
    ht_surface_area = horizontal_tail.area(type='wetted')
    layup_thickness = 0.04/1000 #m (0.04mm)

    fiberglass_mass = ht_surface_area*fiberglass_density*layup_thickness*fiberglass_layup_epoxy_factor
    ht_fiberglass_massprops = asb.MassProperties(mass = fiberglass_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

    ht_foam_mass = horizontal_tail.xsecs[0].airfoil.area()*foam_density*h_c/10
    ht_foam_massprops = asb.MassProperties(mass = ht_foam_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

    ht_carbon_mass = 2*0.05*h_b*layup_thickness*carbon_fiber_density*carbon_fiber_layup_epoxy_factor #0.05 is estimate for width, 0.0001 is estimate for thickness, 2 for both sides
    ht_carbon_massprops = asb.MassProperties(mass = ht_carbon_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

    ht_mass_props = ht_fiberglass_massprops + ht_foam_massprops + ht_carbon_massprops
    Massprops = Massprops + ht_mass_props

    ## VERTICAL TAIL
    vt_surface_area = vertical_tail.area(type='wetted')

    fiberglass_mass = vt_surface_area*fiberglass_density*layup_thickness*fiberglass_layup_epoxy_factor
    vt_fiberglass_massprops = asb.MassProperties(mass = fiberglass_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

    vt_foam_mass = vertical_tail.xsecs[0].airfoil.area()*foam_density*v_c/10
    vt_foam_massprops = asb.MassProperties(mass = vt_foam_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

    vt_carbon_mass = 2*0.05*v_b*layup_thickness*carbon_fiber_density*carbon_fiber_layup_epoxy_factor #0.05 is estimate for width, 0.0001 is estimate for thickness, 2 for both sides
    vt_carbon_massprops = asb.MassProperties(mass = vt_carbon_mass, x_cg = tail_dis+v_c/2, y_cg = 0, z_cg = 0)

    vt_mass_props = vt_fiberglass_massprops + vt_foam_massprops + vt_carbon_massprops
    Massprops = Massprops + vt_mass_props




    # landing gear
    # landing gear mass assumed to be 500 g
    landing_gear_mass = 0.5
    landing_gear_mass_props = asb.MassProperties(mass = landing_gear_mass, x_cg = 0.0, y_cg = 0, z_cg = -0.2)

    Massprops = Massprops + landing_gear_mass_props


    #### PYLON POSITION DEFINITION
    # pylon mass assumed to be 150 g
    pylon_y_guess = 0.58
    pylon_y_limit = b/2-0.15
    pylon_y = opti.variable(init_guess=pylon_y_guess, lower_bound=0.1, upper_bound=pylon_y_limit, scale=0.1, category="pylon_y", freeze=True)  # Pylon y location
    pylon_mass = 0.130
    pylon_mass_props = asb.MassProperties(mass = pylon_mass, x_cg = 0., y_cg = pylon_y, z_cg = 0) + asb.MassProperties(mass = pylon_mass, x_cg = 0., y_cg = -pylon_y, z_cg = 0)

    Massprops = Massprops + pylon_mass_props


    #### ELECTRONICS POSITION DEFINITION
    #motor
    # motor mass assumed, but location is a variable
    motor_mass = 0.5
    motor_x = opti.variable(init_guess=-0.25, lower_bound=-0.3, upper_bound=0, scale=0.1, category="motor_x")  # Motor x location
    prop1_loc = [motor_x, 0, 0]
    motor_mass_props = asb.MassProperties(mass = motor_mass, x_cg = prop1_loc[0], y_cg = prop1_loc[1], z_cg = prop1_loc[2])

    electronics_mass_props = motor_mass_props

    battery_mass = 0.750
    battery_x = opti.variable(init_guess=-0.2, lower_bound=-0.3, upper_bound=0.3, scale=0.1, category="battery_x")  # Battery x location
    battery_mass_props = asb.MassProperties(mass = battery_mass, x_cg = battery_x, y_cg = 0, z_cg = 0)
    electronics_mass_props = electronics_mass_props + battery_mass_props

    esc_mass = 0.1
    esc_x = opti.variable(init_guess=-0.2, lower_bound=-0.3, upper_bound=0, scale=0.1, category="esc_x")  # ESC x location
    esc_mass_props = asb.MassProperties(mass = esc_mass, x_cg = esc_x, y_cg = 0, z_cg = 0)
    electronics_mass_props = electronics_mass_props + esc_mass_props

    wing_servo_mass = 0.1
    tail_servo_mass = 0.05
    wing_servo_massprops = asb.MassProperties(mass=0)
    tail_servo_massprops = asb.MassProperties(mass = tail_servo_mass, x_cg = tail_dis+h_c/2) + asb.MassProperties(mass = tail_servo_mass, x_cg=tail_dis+h_c/2, z_cg=v_b/2)
    for i in range(2):
        loc = b/4*i+b/8
        wing_servo_massprops = wing_servo_massprops + asb.MassProperties(mass = wing_servo_mass, x_cg = loc) + asb.MassProperties(mass = wing_servo_mass, x_cg=-loc)
    electronics_mass_props = electronics_mass_props + wing_servo_massprops + tail_servo_massprops

    wire_length = (b/2+b/4 + # wing servo wires
                tail_dis * 2 + # tail servo wires
                motor_x*2) #wires to motor
    wire_mass_per_m = 0.012
    wire_mass = wire_length*wire_mass_per_m
    wire_massprops = asb.MassProperties(mass = wire_mass, x_cg = rc) #rc is estimate
    electronics_mass_props = electronics_mass_props + wire_massprops

    Massprops = Massprops + electronics_mass_props

    opti.subject_to([
        motor_x+0.1 < esc_x,
        motor_x+0.1 < battery_x,
    ])

    #### FUSELAGE BOX MASS ESTIMATE
    #fuselage extends for about 5cm from centerline to each direction from centerline of wing
    # fuselage_length = rc
    # fuselage_width = 0.1
    # fuselage_height = 0.1
    # fuselage_density = basswood_density
    # # assume mass is the surface area of the fuselage times the density
    # fuselage_thickness = 0.004

    # fuselage_structural_factor = 0.25 # this attempts to estimate that only some of the surface area is actually structure
    # fuselage_mass = (2*fuselage_length*fuselage_width + 2*fuselage_length*fuselage_height + 2*fuselage_width*fuselage_height)*fuselage_density*fuselage_thickness*fuselage_structural_factor
    # screws_mass = 0.15 # for fuselage to wing + boom + tail + landing gear
    fuselage_mass = 0.27
    fuselage_mass_props = asb.MassProperties(mass = fuselage_mass, x_cg = rc/4, y_cg = 0, z_cg = 0)
    Massprops = Massprops + fuselage_mass_props




    ##### BOOM SIZING

    # boom mass
    # boom mass is calculated by estimating the deflection when the tail has a CL of 0.
    # boom_outer_diameter = opti.variable(init_guess=0.04, lower_bound=0.01, upper_bound=0.1, scale=0.01, category="boom_outer_diameter")  # Outer diameter of the boom
    # boom_inner_diameter = opti.variable(init_guess=0.038, lower_bound=0.001, upper_bound=0.1, scale=0.01, category="boom_inner_diameter")  # Inner diameter of the boom
    # boom_radius = boom_outer_diameter/2
    # carbon_fiber_youngs_modulus = 230e9 * 0.4 #according to ChatGPT, when 45-45, E can be 30% of max
    # boom_length = tail_dis + v_c - motor_x#m
    # boom_mass = carbon_fiber_density*np.pi*(boom_radius**2-(boom_inner_diameter/2)**2)*boom_length
    # x_cg_boom = (tail_dis + v_c + motor_x)/2
    # boom_mass_props = asb.MassProperties(mass = boom_mass, x_cg = x_cg_boom, y_cg = 0, z_cg = 0)

    # Massprops = Massprops + boom_mass_props

    # assumed_tail_CLmax = 1.0
    # assumed_max_veolcity_for_tail = 45 # cruise speed is about ~45 im setting the max displacement at and 10 degrees aoa to be 4 cm
    # force_on_tail = assumed_tail_CLmax*assumed_max_veolcity_for_tail**2*Sh*rho/2

    # boom_I = np.pi*((boom_outer_diameter/2)**4-(boom_inner_diameter/2)**4)/64
    # boom_minor_length = boom_length - rc + motor_x
    # boom_deflection = force_on_tail*boom_minor_length**3/(3*carbon_fiber_youngs_modulus*boom_I)
    # opti.subject_to([boom_deflection < 0.04,
    #                 boom_outer_diameter > boom_inner_diameter,
    #                 boom_outer_diameter-boom_inner_diameter > 0.002
    #                 ])
    ## tail boom
    # tail_boom = asb.Fuselage('Tail Boom',
    #                         xsecs = [
    #                             asb.FuselageXSec(
    #                                 xyz_c=[motor_x, 0, 0],
    #                                 xyz_normal=[-1, 0, 0],
    #                                 radius = boom_radius,
    #                                 shape = 2,
    #                             ),
    #                             asb.FuselageXSec(
    #                                 xyz_c=[tail_dis+v_c, 0, 0],
    #                                 xyz_normal=[-1, 0, 0],
    #                                 radius = boom_radius,
    #                                 shape = 2,
    #                             ),
    #                         ]
    # )

    # square boom

    #aluminum properties
    # """prototype 1 aluminum boom"""
    # material_yield_strength = 276e6 #Pa #aluminum
    # material_density = 2710 #kg/m^3
    # material_youngs_modulus = 70e9 #Pa #this is aluminum

    #carbon fiber
    material_density = carbon_fiber_density #kg/m^3
    material_youngs_modulus = carbon_fiber_youngs_modulus*0.4 # 40% is just an estimate given 45-45 layup
    # density_per_length = 0.5/1.68

    material_torsional_modulus = material_youngs_modulus/(2*(1+0.7)) #Pa #.7 assumes CF poisson ratio
    boom_outer_h = opti.variable(init_guess=0.028, lower_bound=0.01, upper_bound=0.1, scale=0.01, category="boom_outer_diameter", freeze = True)  # Outer diameter of the boom
    boom_inner_h = opti.variable(init_guess=0.0225, lower_bound=0.001, upper_bound=0.1, scale=0.01, category="boom_inner_diameter", freeze = True)  # Inner diameter of the boom
    boom_length = tail_dis + v_c - motor_x#m
    boom_mass = material_density*(boom_outer_h**2-boom_inner_h**2)*boom_length * 0.59 # adding factor of 0.59 to correct based on measurements
    # boom_mass = density_per_length*boom_length
    x_cg_boom = (tail_dis + v_c + motor_x)/2
    boom_mass_props = asb.MassProperties(mass = boom_mass, x_cg = x_cg_boom, y_cg = 0, z_cg = 0)

    Massprops = Massprops + boom_mass_props
    opti.subject_to(boom_length == 1.7)

    assumed_tail_CLmax = 1.0
    assumed_max_veolcity_for_tail = 45 # cruise speed is about ~45 im setting the max displacement at and 10 degrees aoa to be 4 cm
    force_on_tail = assumed_tail_CLmax*assumed_max_veolcity_for_tail**2*Sh*rho/2

    boom_I = 1/12*(boom_outer_h**4-boom_inner_h**4)
    boom_J = 2/3*(boom_outer_h**4-boom_inner_h**4)
    boom_minor_length = boom_length - rc + motor_x
    boom_deflection = force_on_tail*boom_minor_length**3/(3*material_youngs_modulus*boom_I)
    compressive_stress = force_on_tail*boom_minor_length/(boom_outer_h**2-boom_inner_h**2)

    tail_torsional_force = v_b/2 * 1/2*rho*assumed_max_veolcity_for_tail**2*Sv*assumed_tail_CLmax
    torsional_shear_stress_boom = tail_torsional_force*boom_outer_h/2/(boom_J)
    material_max_shear_stress = 207e6 #Pa #aluminum


    tail_deflection_limit = b*0.02

    print('\n\nBOOM SIZING FROZEN. LENGTH AND SIZE SET TO CURRENT PURCHASED\n\n')
    # opti.subject_to([boom_deflection < tail_deflection_limit,
    #                 boom_outer_h > boom_inner_h,
    #                 # boom_outer_h-boom_inner_h > 0.002,
    #                 compressive_stress < material_yield_strength,
    #                 torsional_shear_stress_boom < material_torsional_modulus,
    #                 ])
    #tail boom
    tail_boom = asb.Fuselage('Tail Boom',
                            xsecs = [
                                asb.FuselageXSec(
                                    xyz_c=[motor_x, 0, 0],
                                    xyz_normal=[-1, 0, 0],
                                    radius = boom_outer_h/2,
                                    shape = 10,
                                ),
                                asb.FuselageXSec(
                                    xyz_c=[tail_dis+v_c, 0, 0],
                                    xyz_normal=[-1, 0, 0],
                                    radius = boom_outer_h/2,
                                    shape = 10,
                                ),
                            ]
    )
    fuselages += [tail_boom]


    #### PAYLOAD MASS DEFINITION
    payload_mass = opti.variable(init_guess=params['payload_mass'], lower_bound=0.1, upper_bound=25, scale=0.1, freeze = True, category="payload_mass")  # Payload mass
    payload_cg = opti.variable(init_guess=0., lower_bound=-0.2, upper_bound=pod_length, scale=0.1, category="payload_cg", freeze=True)  # Payload cg
    payload_mass_props = asb.MassProperties(mass = payload_mass/2, x_cg = payload_cg+pod_length/2, y_cg = pylon_y, z_cg =z_disp_pod) + asb.MassProperties(mass = payload_mass/2, x_cg = payload_cg+pod_length/2, y_cg = -pylon_y, z_cg = z_disp_pod)

    ##### SPAR SIZING
    # wing spar is sized by applying a distributed maximum load factor, and restricting the displacement at the wing tip
    airfoil_thickness = airfoil.max_thickness()
    tip_thickness = airfoil_thickness*tc

    #wing spar sizing
    #stiffness sizing
    # ultimate_load_factor = opti.variable(init_guess=9, lower_bound=9, upper_bound=10, scale=1, category="ultimate_load_factor")  # Ultimate load factor
    # spar_outer_diameter = opti.variable(init_guess=0.029, lower_bound=0.01, upper_bound=0.1, scale=0.01, category="spar_outer_diameter")  # Outer diameter of the spar
    # spar_inner_diameter = opti.variable(init_guess=0.0275, lower_bound=0.001, upper_bound=0.1, scale=0.01, category="spar_inner_diameter")  # Inner diameter of the spar
    # Massprops_spar = Massprops + payload_mass_props + asb.MassProperties(mass = 0.5, x_cg = .25*rc, y_cg = 0, z_cg = 0) #estimating spar mass for spar load factor sizing
    # distributed_load = ultimate_load_factor*Massprops.mass*g/b
    # spar_I = np.pi*(spar_outer_diameter**4-spar_inner_diameter**4)/64
    # tip_deflection = distributed_load*(b/2)**4/(8*carbon_fiber_youngs_modulus*spar_I)
    # opti.subject_to([
    #     tip_deflection < 0.05,
    #     spar_outer_diameter > spar_inner_diameter,
    #     spar_outer_diameter<tip_thickness,
    #     spar_outer_diameter-spar_inner_diameter > 0.002
    #     ])
    # # strength sizing
    # max_shear_stress_carbon = 66e6 #Pa
    # max_compressive_stress = 700e6 * 0.3 #Pa ChatGPT estimates 30% of max for 45-45
    # inner_radius = spar_inner_diameter/2
    # outer_radius = spar_outer_diameter/2
    # compressive_stress = outer_radius/(np.pi*(outer_radius**4-inner_radius**4))*(payload_mass/2*g*ultimate_load_factor*wing_location + 1/2*Massprops.mass/2*g*ultimate_load_factor*(b/2)**2)
    # opti.subject_to(compressive_stress < max_compressive_stress)
    # #frankly i dont understand moments of inertia to do this properly, but for one thing, I approximate the spar as two spars with two cgs?

    # spar_density_per_length = (np.pi*(spar_outer_diameter/2)**2-np.pi*(spar_inner_diameter/2)**2)*carbon_fiber_density#* 1.5 # 1.5 factor for epoxy mass
    # spar_mass = spar_density_per_length*b
    # wing_spar_mass_props = asb.MassProperties(mass = spar_mass/2, x_cg = rc*.25, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_mass/2, x_cg = rc*.25, y_cg = -b/4, z_cg = 0)
    # wing_mass_props = wing_mass_props + wing_spar_mass_props
    # Massprops = Massprops + wing_spar_mass_props

    ###### COMMENT OUT IF CIRCULAR SPAR
    #spars square

    # spar_outer_h = opti.variable(init_guess=1*units.inch, lower_bound=0.01, upper_bound=0.1, scale=0.01, category="spar_outer_diameter", freeze=True)  # Outer diameter of the spar
    # spar_inner_h= opti.variable(init_guess=(1-.063)*units.inch, lower_bound=0.001, upper_bound=0.1, scale=0.01, category="spar_inner_diameter", freeze = True)  # Inner diameter of the spar
    # spar_mass = material_density*(spar_outer_h**2-spar_inner_h**2)*b * 2 # x 2 because idk, it should be twice this?
    # x_cg_spar = rc*.25
    # spar_mass_props = asb.MassProperties(mass = spar_mass, x_cg = x_cg_spar, y_cg = 0, z_cg = 0)
    # distributed_load_per_length = ultimate_load_factor*(Massprops.mass+0.8)/2*g/(b/2)
    # Massprops = Massprops + spar_mass_props

    # spar_I = 1/12*(spar_outer_h**4-spar_inner_h**4)
    # spar_l = b/2 #obviously, the spar is length b, but we put b/2 because its secured at the wing root

    # F_point = payload_mass/2*g*ultimate_load_factor
    # compressive_stress_spar = 12*spar_outer_h/2/(spar_outer_h**4-spar_inner_h**4)*(-distributed_load_per_length*(b/2)**2+F_point*wing_location)
    # displacement_spar_uniform = distributed_load_per_length*(b/2)**4/(8*material_youngs_modulus*spar_I)
    # displacement_spar_point = payload_mass/2*g*ultimate_load_factor*wing_location**2*(3*b/2-wing_location)/(6*material_youngs_modulus*spar_I)
    # tip_deflection = displacement_spar_uniform + displacement_spar_point
    # opti.subject_to([tip_deflection < 0.1,
    #                 spar_outer_h > spar_inner_h,
    #                 # spar_outer_h-spar_inner_h > 0.002,
    #                 compressive_stress < material_yield_strength,
    #                 ])
    # wing_spar_mass_props = asb.MassProperties(mass = spar_mass/2, x_cg = x_cg_spar, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_mass/2, x_cg = x_cg_spar, y_cg = -b/4, z_cg = 0)
    # wing_mass_props = wing_mass_props + wing_spar_mass_props
    # Massprops = Massprops + wing_spar_mass_props



    ##### SANDWICH SPAR SIZING

    # Add constraints
    positive_limit_load_factor = 10
    negative_limit_load_factor = 4

    # Define core material properties
    core_density = balsa_density
    core_shear_strength = balsa_shear_strength

    # Define cap material properties
    cap_material_density = carbon_fiber_density
    cap_material_yield_strength = carbon_fiber_yield_strength
    cap_material_youngs_modulus = carbon_fiber_youngs_modulus

    spar_width = opti.variable(
        init_guess=12/1000,
        lower_bound= 4/1000,
        upper_bound=20/1000,
        category="spar_width",
        freeze=True
    )  # Width of the spar caps and core

    spar_cap_thickness = opti.variable(
        init_guess=1/1000,
        lower_bound= 0.8/1000,
        upper_bound=5/1000,
        category="spar_cap_thickness",
        freeze=True
    )  # Thickness of the outer skins

    # asb.Airfoil().local_thickness()
    core_thickness_root = wing.xsecs[0].airfoil.local_thickness(x_over_c = 0.25)*rc-spar_cap_thickness*2

    core_thickness_tip = wing.xsecs[-1].airfoil.local_thickness(x_over_c = 0.25)*tc-spar_cap_thickness*2

    # Compute spanwise core thickness variation (linear taper)
    def core_thickness(y):
        eta = 2 * y / b  # Non-dimensional spanwise location (eta = 0 at root, 1 at tip)
        return core_thickness_root * (1 - eta) + core_thickness_tip * eta

    # Mass properties

    spar_carbon_mass_no_epoxy = 2 * spar_width * spar_cap_thickness * b * cap_material_density
    spar_carbon_mass_with_epoxy = spar_carbon_mass_no_epoxy * carbon_fiber_layup_epoxy_factor
    spar_carbon_massprops_no_epoxy = asb.MassProperties(mass = spar_carbon_mass_no_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_carbon_mass_no_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)
    spar_carbon_massprops_epoxy = asb.MassProperties(mass = spar_carbon_mass_with_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_carbon_mass_with_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)

    spar_core_mass = spar_width * (core_thickness_root + core_thickness_tip) / 2 * b * core_density /2 # /2 to fit measurements
    spar_core_massprops = asb.MassProperties(mass = spar_core_mass/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_core_mass/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)

    spar_fiberglass_mass_no_epoxy = (2 * (core_thickness_root+core_thickness_tip)/2 * b)* layup_thickness * fiberglass_density / 2 # /2 to fit measurements
    spar_fiberglass_mass_with_epoxy = spar_fiberglass_mass_no_epoxy * fiberglass_layup_epoxy_factor
    spar_fiberglass_massprops_no_epoxy = asb.MassProperties(mass = spar_fiberglass_mass_no_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_fiberglass_mass_no_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)
    spar_fiberglass_massprops_epoxy = asb.MassProperties(mass = spar_fiberglass_mass_with_epoxy/2, x_cg = rc/4, y_cg = b/4, z_cg = 0) + asb.MassProperties(mass = spar_fiberglass_mass_with_epoxy/2, x_cg = rc/4, y_cg = -b/4, z_cg = 0)

    spar_mass_with_epoxy = spar_carbon_mass_with_epoxy + spar_core_mass + spar_fiberglass_mass_with_epoxy
    spar_mass_no_epoxy = spar_carbon_mass_no_epoxy + spar_core_mass + spar_fiberglass_mass_no_epoxy
    spar_epoxy_mass = spar_mass_with_epoxy - spar_mass_no_epoxy

    spar_mass = spar_carbon_mass_with_epoxy

    # Bending moment of inertia (Equation 16)
    spar_I = (
        spar_width * spar_cap_thickness * (core_thickness_root + spar_cap_thickness)**2
    )  # Only outer skins (caps) contribute significantly

    #positive load factor
    # Distributed load (positive or negative z-direction)
    positive_distributed_load_per_length = positive_limit_load_factor * (Massprops.mass + spar_mass) / b * g  # Uniform load

    # Point load (positive or negative z-direction)
    point_load_positive = payload_mass/2 * g * positive_limit_load_factor

    # negative load factor 
    # Distributed load (positive or negative z-direction)
    negative_distributed_load_per_length = negative_limit_load_factor * (Massprops.mass + spar_mass) / b * g  # Uniform load

    # Point load (positive or negative z-direction)
    point_load_negative = payload_mass/2 * g * negative_limit_load_factor

    # Bending moments for each load case
    # 1. Distributed + Point Load in Positive Z-Direction
    moment_positive = (
        positive_distributed_load_per_length * b**2 / 2 -  # Max moment due to distributed load (root)
        point_load_positive * (b - pylon_y)  # Additional moment due to point load
    )

    # 2. Distributed + Point Load in Negative Z-Direction
    moment_negative = (
        -negative_distributed_load_per_length * b**2 / 2 +  # Max moment due to distributed load (root)
        -point_load_negative * (b - pylon_y)  # Additional moment due to point load
    )

    # Tip deflections for each load case (Equation 19)
    deflection_positive = moment_positive * (b / 2)**2 / (2 * cap_material_youngs_modulus * spar_I)
    deflection_negative = moment_negative * (b / 2)**2 / (2 * cap_material_youngs_modulus * spar_I)

    # Shear stress in the core
    shear_force_positive = positive_distributed_load_per_length * b / 2 - point_load_positive  # Shear force at root
    shear_force_negative = -negative_distributed_load_per_length * b / 2 - point_load_negative  # Shear force at root
    shear_stress_core_positive = shear_force_positive / (spar_width * core_thickness_root)
    shear_stress_core_negative = shear_force_negative / (spar_width * core_thickness_root)

    # Bending stress in spar caps
    bending_stress_positive = moment_positive * (core_thickness_root + spar_cap_thickness) / (2 * spar_I)
    bending_stress_negative = moment_negative * (core_thickness_root + spar_cap_thickness) / (2 * spar_I)

    tip_deflection_limit = 1.82*0.02 # Maximum allowable deflection (stiffness constraint)
    opti.subject_to([
        deflection_positive < tip_deflection_limit,  # Tip deflection for positive load case
        -deflection_negative < tip_deflection_limit,  # Tip deflection for negative load case
        shear_stress_core_positive < core_shear_strength,  # Core shear strength constraint (positive load case)
        shear_stress_core_positive > -core_shear_strength,
        shear_stress_core_negative < core_shear_strength,  # Core shear strength constraint (negative load case)
        shear_stress_core_negative > -core_shear_strength,
        bending_stress_positive < cap_material_yield_strength,  # Cap strength constraint (positive load case)
        bending_stress_positive > -cap_material_yield_strength,
        bending_stress_negative < cap_material_yield_strength,  # Cap strength constraint (negative load case)
        bending_stress_negative > -cap_material_yield_strength,
        core_thickness_root > core_thickness_tip,  # Ensure taper direction is correct
    ])

    # Update mass properties
    x_cg_spar = rc * 0.25
    wing_spar_mass_props = spar_carbon_massprops_epoxy + spar_fiberglass_massprops_epoxy + spar_core_massprops
    wing_mass_props = wing_mass_props + wing_spar_mass_props
    Massprops = Massprops + wing_spar_mass_props


    # sandwich spar
    empty_mass_props = Massprops #with pylons, without weight
    total_mass_props = empty_mass_props + payload_mass_props # full payload
    total_mass = total_mass_props.mass



    """ ------ AERODYNAMICS --------"""
    #### AERODYNAMIC ANALYSIS
    # aerodynamic definition
    pod1 = asb.Fuselage('Pod1',
                        xsecs=[
                            asb.FuselageXSec(
                                xyz_c=[x, 0, 0],
                                xyz_normal=[-1, 0, 0],
                                radius=elliptical_radius(x, pod_length, diameter),
                                shape=2,
                            ) for x in xs
                        ],
                    ).translate([payload_cg, pylon_y, z_disp_pod])
    pod2 = asb.Fuselage('Pod2',
                        xsecs=[
                            asb.FuselageXSec(
                                xyz_c=[x, 0, 0],
                                xyz_normal=[-1, 0, 0],
                                radius=elliptical_radius(x, pod_length, diameter),
                                shape=2,
                            ) for x in xs
                        ],
                    ).translate([payload_cg, -pylon_y, z_disp_pod])
    fuselages += [pod1, pod2]
    wing_location2 = b/4
    z_disp_pod2 = 0.0

    pod_volume = pod1.volume()
    total_fuel_volume = pod_volume*number_of_pods

    #1 prop
    prop_diameter = opti.variable(init_guess=20, lower_bound=6, upper_bound=24, scale=1, category="prop_diameter") #units inch
    normal = [-1, 0, 0]
    num_motor = 1
    # prop_efficiency = 0.65



    prop1 = asb.Propulsor('Motor',
                        xyz_c=prop1_loc,
                        radius=prop_diameter*units.inch/2,
                        xyz_normal = normal,
                        )
    props = [prop1]

    plane_full = asb.Airplane(name='Plane full',
                        xyz_ref= [total_mass_props.x_cg, 0, total_mass_props.z_cg],
                        wings = wings,
                        propulsors=props,
                        fuselages=fuselages,
    )
    plane_empty = asb.Airplane(name='Plane empty',
                        xyz_ref= [empty_mass_props.x_cg, 0, empty_mass_props.z_cg],
                        wings = wings,
                        propulsors=props,
                        fuselages=fuselages,
    )

    m2_velocity = opti.variable(init_guess=params['m2_velocity'], lower_bound=1, upper_bound=50, scale=5, category="m2_velocity", freeze=False)  # Velocity
    #extra drag
    def wheel(velocity, diameter, CD):
        S = np.pi*(diameter/2)**2
        drag = 0.5*rho*velocity**2*CD*S
        return drag
    wheel_diameter = 77.8/1000 #m
    wheel_S = np.pi*(wheel_diameter/2)**2
    CD_landing_gear = 0.52
    drag_landing_gear = 0.5*rho*m2_velocity**2*CD_landing_gear*wheel_S
        #boom

        #fuel pods
        #assuming 0.5 L soda bottle: 
    def fuel_pods(velocity, diameter, num_pods=2):
        CD = 0.26*num_pods
        S = np.pi*(diameter/2)**2
        drag = 0.5*rho*velocity**2*CD*S
        return drag
    diameter = 0.0635 #m
    number_of_pods = 2
    # length = 0.203 #m
    CD_pod = 0.26*number_of_pods # Hoerner 13-16
    S_pod = np.pi*(diameter/2)**2
    drag_pods = 0.5*rho*m2_velocity**2*CD_pod*S_pod

    # propulsion is NOT currently frozen
    propulsion_efficiency = 0.65
    prop_pitch = opti.variable(init_guess=10, lower_bound=5, upper_bound=24, scale=1, category="prop_pitch", freeze=False)  # Pitch
    Kv = opti.variable(init_guess=540, lower_bound=400, upper_bound=2000, scale=100, category="Kv", freeze=False)  # KV
    voltage = 29.6 # 8S battery
    Pmax = opti.variable(init_guess=1760, lower_bound=0, upper_bound=2000, scale=100, category="Pmax", freeze=False)  # Power
    rpm_max = opti.variable(init_guess=20000, lower_bound=5000, upper_bound=24000, scale=1000, category="rpm_max", freeze=False)  # RPM
    max_thrust = 4.392399e-8*rpm_max*prop_diameter**3.5/(prop_pitch**0.5)*4.23333e-4*rpm_max*prop_pitch
    I = Pmax/(voltage*propulsion_efficiency)
    total_energy = 240000
    disk_loading = max_thrust/(np.pi*(prop_diameter*units.inch/2)**2)
    tip_speed = rpm_max/60*(2*np.pi*(prop_diameter*units.inch/2)) #m/s
    opti.subject_to([
        I<100,
        tip_speed/340 < 0.95
        # disk_loading < 500
        ])

    
    ballast = opti.variable(init_guess=0, lower_bound=0, upper_bound=1, scale=1, category="ballast")  # Ballast mass
    ballast_x_full = opti.variable(init_guess=-0.1, lower_bound=-0.2, upper_bound=0.5, scale=0.1, category="ballast_x")  # Ballast cg
    total_mass_props = total_mass_props + asb.MassProperties(mass = ballast, x_cg = ballast_x_full, y_cg = 0, z_cg = 0)
    empty_mass_props = empty_mass_props + asb.MassProperties(mass = ballast, x_cg = ballast_x_full, y_cg = 0, z_cg = 0)
    ###### MISSION ANALYSIS
    turn_load_factor = params['load_factor']
    #mission 2
    thrust_full = opti.variable(init_guess=30, lower_bound=0, upper_bound=100, scale=10, category="thrust_full")
    alpha_full = opti.variable(init_guess=0, lower_bound=-10, upper_bound=10, scale=5, category="alpha_full")  # Angle of attack
    m2_velocity = opti.variable(init_guess=params['m2_velocity'], lower_bound=1, upper_bound=50, scale=5, category="m2_velocity", freeze=False)  # Velocity
    op_point_full = asb.OperatingPoint(velocity=m2_velocity, alpha=alpha_full)
    aero_full = asb.AeroBuildup(plane_full, op_point_full).run_with_stability_derivatives()

    prop_rpm_full = opti.variable(init_guess=18000, lower_bound=0, upper_bound=24000, scale=1000, category="prop_rpm_full") #1/s
    throttle_full = opti.variable(init_guess=1.0, lower_bound=0, upper_bound=1, scale=0.1, category="throttle_full", freeze=False)  # Throttle
    m2_lap_length = opti.variable(init_guess=1000*units.foot*1.5, scale=10, category="m2_lap_length")  # Lap length

    # m2_takeoff_energy = opti.variable(init_guess=0, lower_bound=0, upper_bound=total_energy, scale=1, category="m2_takeoff_energy")  # Takeoff energy
    # m2_cruise_energy = opti.variable(init_guess=0, lower_bound=0, upper_bound=total_energy, scale=1, category="m2_cruise_energy")  # Cruise energy

    # ballast_m2 = opti.variable(init_guess=0, lower_bound=0, upper_bound=10, scale=1, category="ballast_m2")  # Ballast mass
    # ballast_x_full = opti.variable(init_guess=-0.1, lower_bound=-0.2, upper_bound=0.5, scale=0.1, category="ballast_x_full")  # Ballast cg
    # total_mass_props = total_mass_props + asb.MassProperties(mass = ballast_m2, x_cg = ballast_x_full, y_cg = 0, z_cg = 0)

    tip_velocity_full = prop_rpm_full*(prop_diameter*units.inch*np.pi/60/2)
    # SM_full = (aero_full['x_np'] - total_mass_props.x_cg) / wing.mean_geometric_chord()
    # opti.subject_to([
    #     SM_full > 0.1,
    #     SM_full < 0.3
    # ])

    opti.subject_to([
        thrust_full == 4.392399e-8*prop_rpm_full*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_full*prop_pitch-m2_velocity),
        prop_rpm_full == Kv*throttle_full*voltage,  
        thrust_full < Pmax/m2_velocity,
        tip_velocity_full < 343,
        throttle_full == 0.85
    ])

    drag_full = aero_full['D']*2 + fuel_pods(m2_velocity, diameter, num_pods=2) + wheel(m2_velocity, wheel_diameter, CD_landing_gear)
    CD_full = drag_full/(0.5*rho*m2_velocity**2*S)
    opti.subject_to([
        drag_full == thrust_full,
        aero_full['L'] == total_mass_props.mass*g,
    ])
    LD2 = opti.variable(init_guess=params['LD2'], lower_bound=0, upper_bound=20, scale=1, category="LD2", freeze=True)  # Lift-to-drag ratio
    opti.subject_to(LD2 == aero_full['L']/aero_full['D'])
    m2_vstall = np.sqrt(2*total_mass_props.mass*g/(rho*S*1.6))
    avg_speed_takeoff_m2 = 0.9*m2_vstall
    m2_takeoff_max_thrust = 4.392399e-8*prop_rpm_full*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_full*prop_pitch-avg_speed_takeoff_m2)
    m2_takeoff_drag = 0.5*rho*avg_speed_takeoff_m2**2*CD_full*S
    m2_takeoff_accel = (m2_takeoff_max_thrust-m2_takeoff_drag)/total_mass_props.mass
    m2_takeoff_distance = m2_vstall**2/(2*m2_takeoff_accel)
    m2_takeoff_energy = m2_takeoff_distance*m2_takeoff_max_thrust*propulsion_efficiency
    m2_cruise_energy = thrust_full*m2_lap_length*4*propulsion_efficiency
    m2_takeoff_time = m2_takeoff_distance/avg_speed_takeoff_m2
    m2_energy = m2_takeoff_energy + m2_cruise_energy

    opti.subject_to(
        m2_energy < total_energy
    )

    m2_turn_velocity = m2_velocity*1.2
    m2_turning_radius = m2_turn_velocity**2/(g*np.sqrt((turn_load_factor)**2-1))
    m2_distance_for_turn = 2*np.pi*m2_turning_radius*2 #in 1 lap, 2 half turns, + 1 full turn
    m2_time = m2_takeoff_time + m2_lap_length*3/m2_velocity
    opti.subject_to(m2_lap_length == downwind_length*2 + m2_distance_for_turn*2)
    m2_time = m2_lap_length*3/m2_velocity

    mission2_score = payload_mass_props.mass/m2_time

    opti.subject_to(m2_time < 5*60)

    #mission 3
    thrust_empty = opti.variable(init_guess=18, lower_bound=0, upper_bound=100, scale=10, category="thrust_empty")
    alpha_empty = opti.variable(init_guess=3, lower_bound=-10, upper_bound=10, scale=5, category="alpha_empty")  # Angle of attack
    m3_velocity = opti.variable(init_guess=params['m3_velocity'], lower_bound=1, upper_bound=50, scale=5, freeze=False, category="m3_velocity")  # Velocity
    op_point_empty = asb.OperatingPoint(velocity=m3_velocity, alpha=alpha_empty)
    number_of_laps_empty = opti.variable(init_guess=7, lower_bound=1, upper_bound=20, scale=1, freeze=False, category="number_of_laps")
    aero_empty = asb.AeroBuildup(plane_empty, op_point_empty).run_with_stability_derivatives()
    # aero_empty_SM = (aero_empty['x_np'] - empty_mass_props.x_cg) / wing.mean_geometric_chord()
    # opti.subject_to([
    #     aero_empty_SM > 0.1,
    #     aero_empty_SM < 0.3
    # ])

    prop_rpm_empty = opti.variable(init_guess=18000, lower_bound=0, upper_bound=24000, scale=1000, category="prop_rpm") #1/s
    throttle_empty = opti.variable(init_guess=1.0, lower_bound=0, upper_bound=1, scale=0.1, category="throttle", freeze=False)  # Throttle
    m3_lap_length = opti.variable(init_guess=1000*units.foot*1.5, scale=10, category="m3_lap_length")  # Lap length

    tip_velocity_empty = prop_rpm_empty*(prop_diameter*units.inch*np.pi/60/2)


    # ballast_m3 = opti.variable(init_guess=0, lower_bound=0, upper_bound=10, scale=1, category="ballast_m3")  # Ballast mass
    # ballast_x_empty = opti.variable(init_guess=-0.1, lower_bound=-0.2, upper_bound=0.5, scale=0.1, category="ballast_x_empty")  # Ballast cg
    # empty_mass_props = empty_mass_props + asb.MassProperties(mass = ballast_m3, x_cg = ballast_x_empty, y_cg = 0, z_cg = 0)

    opti.subject_to([
        thrust_empty == 4.392399e-8*prop_rpm_empty*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_empty*prop_pitch-m3_velocity),
        prop_rpm_empty == Kv*throttle_empty*voltage,  
        thrust_empty < Pmax/m3_velocity,
        tip_velocity_empty < 343,
    ])

    drag_empty = aero_empty['D']*2 + fuel_pods(m3_velocity, diameter, num_pods=2) + wheel(m3_velocity, wheel_diameter, CD_landing_gear)
    CD_empty = drag_empty/(0.5*rho*m3_velocity**2*S)
    opti.subject_to([
        drag_empty == thrust_empty,
        aero_empty['L'] == empty_mass_props.mass*g,
        number_of_laps_empty*m3_lap_length/m3_velocity < 5*60
    ])
    LD3 = opti.variable(init_guess=params['LD3'], lower_bound=0, upper_bound=20, scale=1, category="LD3", freeze=True)  # Lift-to-drag ratio
    opti.subject_to(LD3 == aero_empty['L']/aero_empty['D'])

    m3_vstall = np.sqrt(2*empty_mass_props.mass*g/(rho*S*1.6))
    avg_speed_takeoff_m3 = 0.9*m3_vstall
    m3_takeoff_max_thrust = 4.392399e-8*prop_rpm_full*prop_diameter**3.5/(prop_pitch**0.5)*(4.23333e-4*prop_rpm_full*prop_pitch-avg_speed_takeoff_m3)
    m3_takeoff_drag = 0.5*rho*avg_speed_takeoff_m3**2*CD_empty*S
    m3_takeoff_accel = (m3_takeoff_max_thrust-m3_takeoff_drag)/total_mass_props.mass
    m3_takeoff_distance = m3_vstall**2/(2*m3_takeoff_accel)
    m3_takeoff_energy = m3_takeoff_distance*m3_takeoff_max_thrust*propulsion_efficiency
    m3_cruise_energy = thrust_empty*m3_lap_length*(number_of_laps_empty+1)*propulsion_efficiency
    m3_takeoff_time = m3_takeoff_distance/avg_speed_takeoff_m3
    opti.subject_to(
        m3_takeoff_energy + m3_cruise_energy < total_energy
    )

    m3_turning_velocity = m3_velocity*1.2
    m3_turning_radius = m3_turning_velocity**2/(g*np.sqrt((turn_load_factor)**2-1))
    m3_distance_for_turn = 2*np.pi*m3_turning_radius*2 #in 1 lap, 2 half turns, + 1 full turn
    opti.subject_to(m3_lap_length == downwind_length*2 + m3_distance_for_turn*2)
    m3_time = number_of_laps_empty*m3_lap_length/m3_velocity + m3_takeoff_time
    opti.subject_to([
        m3_time < 5*60,
                        ])
    bonus = params['bonus'] 
    # glider_mass = 0.249/units.lbm
    glider_mass = params['glider_mass']
    mission3_score = number_of_laps_empty + bonus/glider_mass


    m_empty = opti.variable(init_guess = params['m_empty'], category="m_empty", freeze=False)
    opti.subject_to(m_empty == empty_mass_props.mass) # mass of empty plane
    CLmax = 1.6 #assumed
    # Vstall = 15.7
    # Vstall = opti.variable(init_guess=8, lower_bound=2, upper_bound=15, scale=10, category="Vstall")
    # Vstall = np.sqrt(total_mass_props.mass*g/(0.5*rho*S*CLmax))
    # Vstall = opti.variable(init_guess=15.7, lower_bound=2, upper_bound=25, scale=10, category="Vstall", freeze=True)
    Vmin = opti.variable(init_guess=25, lower_bound=2, upper_bound=25, scale=10, category="Vmin")
    # minimum speed constraint

    #ASSUMING CD0 = 0.05
    CD0 = 0.04
    span_efficiency = 0.85
    # opti.subject_to([
    #     # Vstall == 15.7,
    #     # Vstall**2 == total_mass_props.mass*g/(0.5*rho*S*CLmax)
    #     # 15.7**2 == total_mass_props.mass*g/(.5*rho*b*np.sqrt(CD0*span_efficiency*np.pi*S)),
    #     Vmin**2 == total_mass_props.mass*g/(.5*rho*b*np.sqrt(CD0*span_efficiency*np.pi*S))
    # ])
    mission2_opt = 0.0855
    mission3_opt = 12.2462
    total_score = mission2_score/mission2_opt + mission3_score/mission3_opt
    if mission is None:
        opti.minimize(-(total_score)) # minimize negative, so maximize
    elif mission == 'm2':
        opti.minimize(-(mission2_score))
    elif mission == 'm3':
        opti.minimize(-(mission3_score))

    sol = opti.solve(verbose=False, max_runtime=1.5*60)

    #print results
    # geometry output
    print('\n\n__Geometry__:')
    print("Wing:")
    print(f"Root chord: {round(sol(rc), 2)} [m]")
    print(f"Tip chord: {round(sol(tc), 2)} [m]")
    print(f"Taper ratio: {round(sol(taper), 2)} [-]")
    print(f"Span: {round(sol(b), 2)} [m] limit: {6*units.foot} [m])")
    print(f"AR: {round(sol(AR), 2)} [-]")
    print(f"S: {round(sol(S), 2)}")
    print(f"Ultimate load factor (positive): {round(sol(positive_limit_load_factor), 2)}")
    print(f"Ultimate load factor (negative): {round(sol(negative_limit_load_factor), 2)}")
    # print(f"Sweep: {round(sol(np.degrees(sweep)), 2)} [deg]")
    print(f'Motor x: {round(sol(motor_x), 2)} [m]')
    print(f'Battery x: {round(sol(battery_x), 2)} [m]')
    print(f'ESC x: {round(sol(esc_x), 2)} [m]')
    print(f'Payload cg: {round(sol(payload_cg), 2)} [m]')
    print(f'Pylon y: {round(sol(pylon_y), 2)} [m] (limit: {pylon_y_limit} [m])')
    print(f'Boom deflection: {round(sol(boom_deflection), 3)} [m] (should be under {round(tail_deflection_limit, 3)} [m])')
    print(f'Boom inner diameter: {round(sol(boom_inner_h), 4)} [m]')
    print(f'Boom outer diameter: {round(sol(boom_outer_h), 4)} [m]')
    print(f'Boom length: {round(sol(boom_length), 2)} [m]')
    print(f'Boom torsional shear stress: {round(sol(torsional_shear_stress_boom), 2)} [Pa] (should be under {material_max_shear_stress})')
    # print(f'Spar outer diameter: {round(sol(spar_outer_h), 4)} [m]')
    # print(f'Spar inner diameter: {round(sol(spar_inner_h), 4)} [m]')
    print('_____________________')
    print('Spar values:')
    print(f'Spar cap thickness: {round(sol(spar_cap_thickness), 4)} [m]')
    print(f'Spar core width: {round(sol(spar_width), 4)} [m]')
    print(f'Positive wing deflection: {round(sol(deflection_positive), 3)} [m] (should be under {round(tip_deflection_limit, 3)} [m])')
    print(f'Negative wing deflection: {round(sol(deflection_negative), 3)} [m] (should be under {round(tip_deflection_limit, 3)} [m])')
    print(f'Shear stress (positive): {round(abs(sol(shear_stress_core_positive)), 2)} [Pa] (should be under {core_shear_strength})')
    print(f'Shear stress (negative): {round(abs(sol(shear_stress_core_negative)), 2)} [Pa] (should be under {core_shear_strength})')
    print(f'Bending stress (positive): {round(abs(sol(bending_stress_positive)), 2)} [Pa] (should be under {cap_material_yield_strength})')
    print(f'Bending stress (negative): {round(abs(sol(bending_stress_negative)), 2)} [Pa] (should be under {cap_material_yield_strength})')
    print("\n____________________")

    # tail output
    print("\nTail:")
    print(f"Tail distance: {round(sol(tail_dis), 2)}")
    print(f"Vertical tail span: {round(sol(v_b), 2)}")
    print(f"Vertical tail chord: {round(sol(v_c), 2)}")
    print(f"Horizontal tail span: {round(sol(h_b), 2)}")
    print(f"Horizontal tail V: {round(sol(H_tail_volume), 2)} (should be 0.3 < Vh < 0.6)")
    print(f"Vertical Tail V: {round(sol(V_tail_volume), 2)} (should be 0.02 < Vv < 0.05)")
    print("\n____________________")

    # propulsion
    print("\n__Propulsion__:")
    print(f"Current: {round(sol(I), 2)} [A]")
    print(f"KV: {round(sol(Kv), 2)} [rpm/V]")
    print(f"Power(max): {round(sol(Pmax), 2)} [W]")
    print(f'Pitch: {round(sol(prop_pitch), 2)} [inch?]')
    print(f"Prop diameter: {round(sol(prop_diameter), 2)} [inch]")
    print(f"Propulsion efficiency: {round(sol(propulsion_efficiency), 2)} [-]")
    print(f'RPM max: {round(sol(rpm_max), 2)} [rpm]')
    print(f'Max disk loading: {round(sol(disk_loading), 2)} [N/m^2]')
    print("\n____________________")

    # objective
    print("\n__Objective__:")
    print(f'Total score: {round(sol(total_score), 2)}')
    print("\n____________________\n")
    print(f'Mission 2: {round(sol(mission2_score), 4)} Percent of estimated optimum {round(sol(mission2_score)/mission2_opt*100, 2)}%')
    print(f'Velocity: {round(sol(m2_velocity), 2)} [m/s]')
    print(f'Alpha: {round(sol(alpha_full), 2)} [deg]')
    print(f'CL: {round(sol(aero_full["CL"]), 2)}')
    print(f'CD: {round(sol(CD_full), 2)}')
    print(f'Reynolds: {round(sol(op_point_full.reynolds(rc)), 2)}')
    print(f'Thrust: {round(sol(thrust_full), 2)} [N]')
    print(f'Throttle: {round(sol(throttle_full), 2)}')
    print(f'RPM: {round(sol(prop_rpm_full), 2)}')
    # print(f'SM: {round(sol(SM_full), 2)}')
    print(f'Turn radius: {round(sol(m2_turning_radius), 2)} [m]')
    print(f"Total mass: {round(sol(total_mass_props.mass), 2)} [kg]")
    print(f"Payload mass: {round(sol(payload_mass_props.mass), 2)} [kg]")
    print(f"Empty mass: {round(sol(empty_mass_props.mass), 2)} [kg]")
    print(f"Flight time: {round(sol(m2_time), 2)} [sec]")
    print(f"Flight time per lap: {round(sol(m2_time)/3, 2)} [sec]")
    print(f"Pod Volume: {round(sol(total_fuel_volume), 2)} [m^3]")
    print(f"Required fuel density: {round(sol(payload_mass)/sol(total_fuel_volume), 2)} [kg/m^3]")
    print(f'Energy used on takeoff: {round(sol(m2_takeoff_energy), 2)} [J] {round(sol(m2_takeoff_energy)/total_energy*100, 2)}%')
    print(f'Energy used in cruise: {round(sol(m2_cruise_energy), 2)} [J] {round(sol(m2_cruise_energy)/total_energy*100, 2)}%')
    print(f'Takeoff distance: {round(sol(m2_takeoff_distance), 2)} [m]')
    # print("Ballast: ", round(sol(ballast_full), 2))
    # print("Ballast x: ", round(sol(ballast_x_full), 2))
    print(f'Cg location: {round(sol(total_mass_props.x_cg), 2)}')
    print(f'Neutral point: {round(sol(aero_full["x_np"]), 2)}')
    print("\n____________________\n")
    print(f'Mission 3: {round(sol(mission3_score), 4)} Percent of estimated optimum {round(sol(mission3_score)/mission3_opt*100, 2)}%')
    print(f"Total laps: {round(sol(number_of_laps_empty), 2)}")
    print(f"Flight time: {round(sol(m3_time), 2)} [sec]")
    print(f"Flight time per lap: {round(sol(m3_time)/sol(number_of_laps_empty), 2)} [sec]")
    print(f"Mass: {round(sol(empty_mass_props.mass), 2)} [kg]")
    print(f"Velocity: {round(sol(m3_velocity), 2)} [m/s]")
    print(f"Alpha: {round(sol(alpha_empty), 2)} [deg]")
    print(f"CL: {round(sol(aero_empty['CL']), 2)}")
    print(f"CD: {round(sol(aero_empty['CD']), 2)}")
    print(f"Reynolds: {round(sol(op_point_empty.reynolds(rc)), 2)}")
    print(f"Thrust: {round(sol(thrust_empty), 2)} [N]")
    print(f'Throttle: {round(sol(throttle_empty), 2)}')
    print(f'RPM: {round(sol(prop_rpm_empty), 2)}')
    # print(f'SM: {round(sol(aero_empty_SM), 2)}')
    print(f"Turn radius: {round(sol(m3_turning_radius), 2)} [m]")
    print(f"Energy used on takeoff: {round(sol(m3_takeoff_energy), 2)} [J] {round(sol(m3_takeoff_energy)/total_energy*100, 2)}%")
    print(f"Energy used in cruise: {round(sol(m3_cruise_energy), 2)} [J] {round(sol(m3_cruise_energy)/total_energy*100, 2)}%")
    print(f'Takeoff distance: {round(sol(m3_takeoff_distance), 2)} [m]')
    # print("Ballast: ", round(sol(ballast_empty), 2))
    # print("Ballast x: ", round(sol(ballast_x_empty), 2))
    print(f'Cg location: {round(sol(empty_mass_props.x_cg), 2)}')
    print(f'Neutral point: {round(sol(aero_empty["x_np"]), 2)}')

    print("\n____________________\n")
    print('\n\n__Mass Breakdown__:')
    empty_mass_props = sol(empty_mass_props)
    print('Everything is divided by total empty mass')
    print(f"Wing mass: {round(sol(wing_mass_props.mass), 2)} [kg], {round(sol(wing_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
    print(f"    Rib mass: {round(sol(wing_rib_mass_props.mass), 2)} [kg], {round(sol(wing_rib_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
    print(f"    Spar mass: {round(sol(wing_spar_mass_props.mass), 2)} [kg], {round(sol(wing_spar_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
    print(f"         Cap mass: {round(sol(spar_carbon_massprops_epoxy.mass), 2)} [kg], {round(sol(spar_carbon_massprops_epoxy.mass)/sol(wing_spar_mass_props.mass)*100, 2)} [% of spar]")
    print(f"              Cap epoxy: {round(sol(spar_carbon_massprops_epoxy.mass)-sol(spar_carbon_massprops_no_epoxy.mass), 2)} [kg],")
    print(f"         Fiberglass mass: {round(sol(spar_fiberglass_massprops_epoxy.mass), 2)} [kg], {round(sol(spar_fiberglass_massprops_epoxy.mass)/sol(wing_spar_mass_props.mass)*100, 2)} [% of spar]")
    print(f"              Fiberglass epoxy: {round(sol(spar_fiberglass_massprops_epoxy.mass)-sol(spar_fiberglass_massprops_no_epoxy.mass), 2)} [kg],")
    print(f"         Core mass: {round(sol(spar_core_massprops.mass), 2)} [kg], {round(sol(spar_core_massprops.mass)/sol(wing_spar_mass_props.mass)*100, 2)} [% of spar]")
    print(f"         Total spar epoxy mass: {round(sol(spar_epoxy_mass), 2)} [kg], {round(sol(spar_epoxy_mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing] (includes epoxy)")
    print(f"    Monokote mass: {round(sol(wing_monokote_mass_props.mass), 2)} [kg], {round(sol(wing_monokote_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
    print(f"    Carbon leading edge mass: {round(sol(wing_carbon_leading_edge_mass_props.mass), 2)} [kg], {round(sol(wing_carbon_leading_edge_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
    print(f"    Flap mass: {round(sol(flap_mass_props.mass), 2)} [kg], {round(sol(flap_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
    print(f"    Aileron mass: {round(sol(aileron_mass_props.mass), 2)} [kg], {round(sol(aileron_mass_props.mass)/sol(wing_mass_props.mass)*100, 2)} [% of wing]")
    print(f"Vertical tail mass: {round(sol(vt_mass_props.mass), 2)} [kg], {round(sol(vt_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
    print(f"    Foam mass: {round(sol(vt_foam_massprops.mass), 2)} [kg], {round(sol(vt_foam_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    print(f"    Fiberglass mass: {round(sol(vt_fiberglass_massprops.mass), 2)} [kg], {round(sol(vt_fiberglass_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    print(f"    Carbon mass: {round(sol(vt_carbon_massprops.mass), 2)} [kg], {round(sol(vt_carbon_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    # print(f"    Rib mass: {round(sol(vt_rib_mass_props.mass), 2)} [kg], {round(sol(vt_rib_mass_props.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    # print(f"    Spar mass: {round(sol(vt_spar_mass_props_tail.mass), 2)} [kg], {round(sol(vt_spar_mass_props_tail.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    # print(f"    Monokote mass: {round(sol(vt_monokote_mass_props.mass), 2)} [kg], {round(sol(vt_monokote_mass_props.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    print(f"Horizontal tail mass: {round(sol(ht_mass_props.mass), 2)} [kg], {round(sol(ht_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%] (includes epoxy)")
    print(f"    Foam mass: {round(sol(ht_foam_massprops.mass), 2)} [kg], {round(sol(ht_foam_massprops.mass)/sol(ht_mass_props.mass)*100, 2)} [% of horizontal tail]")
    print(f"    Fiberglass mass: {round(sol(ht_fiberglass_massprops.mass), 2)} [kg], {round(sol(ht_fiberglass_massprops.mass)/sol(ht_mass_props.mass)*100, 2)} [% of horizontal tail]")
    print(f"    Carbon mass: {round(sol(ht_carbon_massprops.mass), 2)} [kg], {round(sol(ht_carbon_massprops.mass)/sol(vt_mass_props.mass)*100, 2)} [% of vertical tail]")
    # print(f"    Rib mass: {round(sol(ht_rib_mass_props.mass), 2)} [kg], {round(sol(ht_rib_mass_props.mass)/sol(horizontal_tail_mass_props.mass)*100, 2)} [% of horizontal tail]")
    # print(f"    Spar mass: {round(sol(ht_spar_mass_props_tail.mass), 2)} [kg], {round(sol(ht_spar_mass_props_tail.mass)/sol(horizontal_tail_mass_props.mass)*100, 2)} [% of horizontal tail]")
    # print(f"    Monokote mass: {round(sol(ht_monokote_mass_props.mass), 2)} [kg], {round(sol(ht_monokote_mass_props.mass)/sol(horizontal_tail_mass_props.mass)*100, 2)} [% of horizontal tail]")
    print(f"Landing Gear mass: {round(sol(landing_gear_mass_props.mass), 2)} [kg], {round(sol(landing_gear_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
    print(f"Pylon mass: {round(sol(pylon_mass_props.mass), 2)} [kg], {round(sol(pylon_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
    print(f"Boom mass: {round(sol(boom_mass), 2)} [kg], {round(sol(boom_mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
    print(f"Electronics mass: {round(sol(electronics_mass_props.mass), 2)} [kg], {round(sol(electronics_mass_props.mass)/sol(empty_mass_props.mass)*100, 2)} [%]")
    print(f"    Motor mass: {round(sol(motor_mass), 2)} [kg], {round(sol(motor_mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
    print(f"    Battery mass: {round(sol(battery_mass), 2)} [kg], {round(sol(battery_mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
    print(f"    ESC mass: {round(sol(esc_mass), 2)} [kg], {round(sol(esc_mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
    print(f"    Wing servo mass: {round(sol(wing_servo_massprops.mass), 2)} [kg], {round(sol(wing_servo_massprops.mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
    print(f"    Tail servo mass: {round(sol(tail_servo_massprops.mass), 2)} [kg], {round(sol(tail_servo_massprops.mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
    print(f"    Servo wires (and other): {round(sol(wire_massprops.mass), 2)} [kg], {round(sol(wire_massprops.mass)/sol(electronics_mass_props.mass)*100, 2)} [% of electronics]")
    print(f"Payload mass: {round(sol(payload_mass_props.mass), 4)} [kg], {round(sol(payload_mass)/sol(total_mass_props.mass)*100, 2)} [%]")
    print(f"Total mass: {round(sol(total_mass_props.mass), 2)} [kg]")
    print(f"Payload mass fraction: {round(sol(payload_mass_props.mass)/sol(total_mass_props.mass)*100, 2)} [%]")

    return_params = {
        'load_factor': sol(turn_load_factor),
        'payload_mass': sol(payload_mass_props.mass),
        'bonus': sol(bonus),
        'glider_mass': sol(glider_mass),
        'm2': sol(mission2_score),
        'm3': sol(mission3_score),
        'm2_velocity': sol(m2_velocity),
        'm3_velocity': sol(m3_velocity),
        'S': sol(S),
        'taper': sol(taper),
        'M_dry': sol(empty_mass_props.mass),
        'LD2': sol(LD2),
        'LD3': sol(LD3),
        'm_empty': sol(m_empty),

    }
    return return_params
    plane_full = sol(plane_full)
    plane_full.draw_three_view()

    # possible_point_masses = empty_mass_props.generate_possible_set_of_point_masses()
    # print()
    # print(possible_point_masses)
    # for i, mass in enumerate(possible_point_masses):
    #     print(i+1)
    #     print(mass.mass)
    #     print([mass.x_cg, mass.y_cg, mass.z_cg])
    #     print()

    # plane_empty = sol(plane_empty)
    # plane_empty_without_pylons = sol(plane_empty_without_pylons)

    # #mission 2

    avl = asb.AVL(airplane=plane_full, op_point=op_point_full, working_directory='avl_runs')
    avl.write_avl('m2')
    full_mass_props = sol(total_mass_props)  
    full_mass_props.export_AVL_mass_file('m2.mass')
    # xflr5 = plane_full.export_XFLR5_xml(filename='m2', mass_props=full_mass_props)
    # #mission 3
    # avl = asb.AVL(airplane=plane_empty, op_point=op_point_empty, working_directory='avl_runs')
    # avl.write_avl('m3')
    # empty_mass_props = sol(empty_mass_props)
    # empty_mass_props.export_AVL_mass_file('m3.mass')


def generate_sensitivity_csv(parameter_name, results_list, directory_name, csv_filename):
    """
    Save sensitivity analysis results to a CSV file in a subdirectory.

    Args:
        parameter_name (str): Name of the parameter being analyzed (for logging purposes).
        results_list (list): List of lists, where each inner list contains [parameter_value, m2_score, m3_score].
        directory_name (str): Name of the subdirectory to save the CSV file.
        csv_filename (str): Name of the CSV file.
    """
    # Ensure the subdirectory exists
    os.makedirs(directory_name, exist_ok=True)

    # Full path to the CSV file
    csv_filepath = os.path.join(directory_name, csv_filename)

    # Define the CSV header
    fieldnames = ['parameter_value', 'm2_score', 'm3_score']

    # Write results to the CSV file
    with open(csv_filepath, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(fieldnames)  # Write the header

        # Write each result row
        for result in results_list:
            writer.writerow(result)

    print(f"Sensitivity analysis for {parameter_name} saved to {csv_filepath}")


def scaled_params(params, param, sensitivity_percent = 20, N = 10):
    """ Returns a list of parameter dictionaries, in each dictionray the selected parameter is sclaed by a sensitivity factor."""
    sensitivity = sensitivity_percent / 100
    params_list = []
    # if param == 'bonus': 
    #     vals = [0, 1, 2.5]
    #     for i in range(3):
    #         new_params = params.copy()
    #         new_params[param] = params[param] = vals[i]
    #         params_list.append(new_params)
    # else:
    for i in range(N):
        scale_factor = 1 + sensitivity * (2 * i / (N - 1) - 1)
        new_params = params.copy()
        new_params[param] = params[param] * scale_factor
        params_list.append(new_params)
    return params_list

if __name__ == "__main__":
    nominal = {
    'load_factor': 6,
    'payload_mass': 10,
    'bonus': 2.5, 
    'glider_mass': 0.100/units.lbm,
    'm2_velocity': 31., 
    'm3_velocity': 31., 
    'S': 0.9,
    'taper': 0.7,
    'LD2': 6.25*0.8,
    'LD3': 8,
    'm_empty': 4.95/0.8
    # 'm3_Nlap': 7.
    }
    mission_analysis = {
        'load_factor': ['m2', 'm3'],
        'payload_mass': ['m2'],
        'bonus': ['m3'], 
        'glider_mass': ['m3'],
        'm2_velocity': ['m2'], 
        'm3_velocity': ['m3'], 
        'S': ['m2', 'm3'],
        'taper': ['m2', 'm3'],
        'm3_Nlap': ['m3'], 
        'LD2': ['m2'],
        'LD3': ['m3'],
        'm_empty': ['m2', 'm3'],
    }
    m2, m3 = True, True
    incomplete = ['LD2', 'LD3', 'load_factor', 'payload_mass', 'bonus', 'glider_mass']
    N = 5
    sensitivity_percent = 20
    for parameter in incomplete:
        print(f'Generating sensitivity for {parameter}')
        params = scaled_params(nominal, parameter, sensitivity_percent, N)  # Generate scaled parameters
        current_param_result = []

        # Initialize progress bar
        with alive_bar(len(params), title=f"Analyzing sensitivity of {parameter}...", bar=custom) as bar:
            for sensitivity_param in params:
                # Optimize for both missions
                try:
                    m2_score = None
                    m3_score = None
                    if m2:
                        if 'm2' in mission_analysis[parameter]:
                            # Run optimization for Mission 2
                            result_m2 = opt(sensitivity_param, mission='m2')
                            m2_score = result_m2['m2']
                    if m3:
                        if 'm3' in mission_analysis[parameter]:
                            # Run optimization for Mission 3
                            result_m3 = opt(sensitivity_param, mission='m3')
                            m3_score = result_m3['m3']
                    current_param_result.append([sensitivity_param[parameter], m2_score, m3_score])
                except Exception as e:
                    print(f"Error with parameter {parameter}: {sensitivity_param}. Error: {e}")
                    current_param_result.append([sensitivity_param[parameter], None, None])
                bar()

            # Save the results to a CSV file
        csv_filename = f'sensitivity2_{parameter}.csv'
        directory_name = f'csvs'
        generate_sensitivity_csv(parameter, current_param_result, directory_name, csv_filename)
            
