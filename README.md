# DBF 2024-2025 
This repo showcases the final optimization code for initial sizing of the MIT DBF 2025 plane: AdaSOARus. The code has undergone several iterations. The optimizer contained here was written after the first/second prototype and test flight. 

## How Should This Repo Be Used?
There's a major misconception about the purpose of an MDO code like this, which often leads to quite undesirable or otherwise misadvised design choices. The primary purpose of a code like this is to allow an engineer to explore the design space and get a feel for the objective function. Importantly, you want to develop an understanding of how changing the variables and their constraints changes the convergence of the 'optimized' aircraft.

What do we actually mean by the word 'optimized'? Here's a big point to get across: 'optimized' does not mean that this is the optimal aircraft for the given mission. The converged aircraft here happens to be the 'optimal' design for the design problem we have posed to it. This is often *very* different from the actual objective. The point here is: do not take optimizer results as a godsend. They're not. If the converged result feels wrong, it likely is. 

A quick recap of how we used this code, as an example:

When we defined the original problem, our aircraft converged to a very quick, very large aircraft (wing area of $1.3m^2$! -- much larger than we're used to). However, after testing some constraints, we noticed that constraining S==0.9, the objective score decreased by a fraction of a percent. After a discussion, we determined the conservative solution would be to build a slightly smaller aircraft and carry a somewhat lighter payload; So we designed an aircraft with S=$0.9m^2$ (after refining our models with testing data, our optimal wing area converged to $0.89m^2$, so we were quite close!).

I mention this story because this is a perfect example of how this code should be used. We have a design unknown: wing area. We want to explore what a "good" wing area is, and how sensitive our performance is to this variable. Using this information, we can use our own design judgement to design a better aircraft than designed by optimizer alone. I hope this humble story of a small DBF team (who still finished very poorly because of structures, not code!), helps you enter the lovely world of vehicle design optimization.

## Quick How-To:
### Installation

1. **Clone the Repository**

   Use the following command to clone this repository to your local machine:
   ```bash
   git clone https://github.com/tarasykm/DBF25-CodeCache.git
   ```

2. **Set Up a Virtual Environment using .yaml**

   Create and activate a virtual environment to avoid dependency conflicts:
   ```terminal
   conda env create -f env.yaml
   conda activate env
   ```

### Usage
To run a single optimization, use optp2.py.

From the last update of README, this is optp2.py -- the iteration after the first flight test, first committed Jan 10th. 

You can run the optimization simply with:

``python optp2.py``

To change the optimization procedure or constraints, go to the source file and change it.

For help navigating/understanding Aerosandbox, navigate to https://github.com/peterdsharpe/AeroSandbox.

The tutorial section is pretty great for understanding how the library works. 

Otherwise, feel free to reach out to me and ask any questions, you know how to reach me :)

## Files

### optp2.py
This is the main optimization code. It's somewhat poorly structured, I apologize, I got better at organizing repos after this. The main goal however is to just go through and see how different parts work. To run: just run the script in your IDE or in your terminal run 'python optp2.py'

### airfoil_selection.py
This file was used for airfoil evaluation and selection. The code takes an objective function, which is a combination of CLmax, CL/CD at a given CL, x tripping point location and airfoil area to calculate an objective score. This score then ranks airfoils. It can also plot their performance, and output the top N performing airfoils to a .txt.

## UPDATES:
### Feb 4th:

Edited airfoil selection. This program is now fully fucntional. Functions are basically outlined at the bottom, I also have a few csvs that have saved some tentaive evaluations. Essentially, its all a question of howyou weigh the different parameters.

The program compiles a score for every airfoil in the UIUC database given a few analyses. 

1. CL/CD - this is the most basic analysis but also among the most important. This evaluates the lift to drag ratio of the airfoil at given CLs at the reynolds number the aircraft will fly at. If you go to the document you'll see this gets the highest weight, but to be honest this is among the most important anyway. 

2. CM - Ideally, airfoils have a CM relatively close to 0 to help stability, however, neuralfoil has trouble estimating this at times so the weight is left low. 

3. Top Xtr - This is the tripping point on the top surface of the airfoil, ie when the flow begins to go turbulent. Ideally, at our flight regime we would want little to no tripping. 

4. Bot Xtr - Same as before, but bottom surface, this has a slightly lower weight though. 

5. CLmax - Since we have no takeoff requirement, high CL is not strictly necessary, however, it doesn't really hurt either. Will also decrease the stall speed, making landings easier.

6. Area - ribs are one of the heavier components on our aircraft, so this model penalizes higher airfoil areas.

If we assume a final mass of ~ 15kg (last flight test was 10kg), the CL for mission 2 and 3 is between 0.1 to 0.3. You are welcome to try other CLs, but this is our current regime. 

### Feb 3rd:
Edited mass model given mass from prototype 2. **THIS IS INCOMPLETE**. 

Added 2 new files:
1. airfoil_selection.py:
   Goal: run xfoil on **every** airfoil in the UIUC database, and evaluate based on an objective function which weighs CL/CD, CM and trip point (ideally also CLmax). However, this currently does not handle XFOIL errors well, as a result, most airfoils are not properly evaluated. If you'd like to gve it a shot, go for it, would like some help. XFOIL is just finnicky. Potential fix: update to use neuralfoil instead, however, this means you need to set up an optimization, as neuralfoil accepts aoa inputs, not CL. 

2. datedit.py:
   This is a general useful program. Given an airfoil, an xhinge point and an angle, it will return a new dat file for the control surface of a given airfoil, with the max deflection angle you want cut into the airfoil. This is intended to make control surfaces using the new foam cutter. Depending on how you define it, this may not work with the old foam cutter as it struggles to identify a leading edge. 

### Jan 12th:
The mass model is reworked in optp2.py. Instead of blanket "structural mass" bump, each component gets individual bumps for epoxy. 

**Note**: its set up for the current build. Many variables that should otherwise be free are constrained to the vehicle we are currently constrained. Notably, spar has the dimensions we built (although the model guesses these are 2-3 times too much), tail is constrained to be like the last. S is constrained to be 0.9 and taper > 0.7.

Another significant change: glider mass is now assumed to be **125g**, to better assess the weight of M2 vs M3. 

The tail is adjusted from full built up to laid up foam. 

Mass model also now includes the following components:
 - Servos
 - Servo wires
 - Fuselage (albeit its a constant)

All the material properties are shifted to the top. 
### Jan 10th:
opt.py will no longer be edited, changes after first flight test are applied to optp2.py. These changes include:
 - Taper
 - Sandwich spar
 - CF boom


Results seem stable, but overzealous on taper. I think it has to do with an overestimate of rib mass, but could aslo be induced drag. Could also be that a 0.1 taper ratio is optimal "technically" disregarding manufacturing difficulty, in which case, the lower bound should be raised due to manufacturing considerations.
