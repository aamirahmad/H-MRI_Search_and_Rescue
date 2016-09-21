#!/usr/bin/python3.4

import itertools
import random
import datetime
import subprocess
import sys
import ast, getopt, types
def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")




def parse_list(argv):
    arg_dict={}
    switches={'li':list,'di':dict,'tu':tuple}
    singles=''.join([x[0]+':' for x in switches])
    long_form=[x+'=' for x in switches]
    d={x[0]+':':'--'+x for x in switches}
    try:
        opts, args = getopt.getopt(argv, singles, long_form)
    except getopt.GetoptError:
        print ("bad arg")
        sys.exit(2)

    for opt, arg in opts:
        if opt[1]+':' in d: o=d[opt[1]+':'][2:]
        elif opt in d.values(): o=opt[2:]
        else: o =''
        #print (opt, arg,o)
        if o and arg:
            arg_dict[o]=ast.literal_eval(arg)

        if not o or not isinstance(arg_dict[o], switches[o]):
            print (opt, arg, " Error: bad arg")
            sys.exit(2)

    #print(arg_dict)
    #for e in arg_dict:
    #    print (e, arg_dict[e], type(arg_dict[e]))        
    return (arg_dict)
    
if __name__ == "__main__":
    # experiment setup variations
    ROBOTS=[2,5]
    MODES=["full_control","semi_auto_control","full_autonomous"]
    MAPS=["arena_HKT_1.world","arena_HKT_2.world"," arena_HKT_3.world"]
    MAPS=list(range(1,19))
    experiment_log = "/home/dementor/traverse_experiment_log/experiment_master_log_"
    log_file = experiment_log  + str(datetime.datetime.now().isoformat()).replace(":","_")

    if len(sys.argv) == 1:
        s = [ROBOTS,MODES]
        # create all permutations
        exp_setups =  list(itertools.product(*s))
        # create desired number of repeats
        exp_setups = list(itertools.chain.from_iterable(itertools.repeat(x, 3) for x in exp_setups))
        # randomize
        random.shuffle(exp_setups)
        print(exp_setups)
        while query_yes_no("reshuffle?"):
        	random.shuffle(exp_setups)
        	print(exp_setups)
        	pass
        # randomize the order of the map
        random.shuffle(MAPS)
    
        experiment_counter = 1 
    elif len(sys.argv) > 1:
        #print()
        argv = sys.argv
        settings = parse_list(sys.argv[1:])
        exp_setups = settings['li']
        used_maps = settings['tu']
    
        maps = range(1,19)
        maps = [x for x in maps if x not in used_maps]
        for i in range (len(used_maps)):
            exp_setups.pop()
        experiment_counter = 18 - len(exp_setups) + 1
        MAPS = maps 
        print(exp_setups)
        print(experiment_counter)
    
    while exp_setups:
    	if ((experiment_counter % 6) == 1) and (experiment_counter > 1):
    		print("\n\n \t \t ================= TAKE A BREAK =============== \n\n")
    	setup = exp_setups.pop()
    	current_map = MAPS.pop()
    	print("Experiment number",experiment_counter)
    	while not query_yes_no("Did you start the roscore on cyb03?"):
    		pass
    	print("Good! \n Now start gazebo on cyb17")
    	print("DISPLAY=:0.0 roslaunch traverse_gazebo arena.launch arena:=arena_HKT_"+str(current_map)+".world")
    	while not query_yes_no("Did you start gazebo on cyb17?"):
    		pass
    	print("Perfect!")
    	print("Execute this command to start experiment in a spereate shell on cyb03")
    	print( './setup_exp.sh', setup[0], setup[1], "arena_HKT_"+str(current_map)+".world")
    	while not query_yes_no("Ok?"):
    		pass
    	print("Now start the blender vr on cyb06")
    	print("DISPLAY=:0.0 blender2.76 traverse_blend_vr/blender/blend_files/traverse_vr_"+str(setup[1])+".blend\n")
    	print("When octomap is running you can start the auto_convert.sh script on cyb06")
    	while not query_yes_no("All set?"):
    		pass
    	# wait for enter
    	with open(log_file, "a") as myfile:
    		print("Writing to logfile",log_file)
    		log_str = str(experiment_counter)+ ","+ str(datetime.datetime.now())+ ", " + str(setup[0]) + ", "+ str(setup[1]) +  ", arena_HKT_"+str(current_map)+".world" "\n"
    		myfile.write(log_str)
    	print("Now you can wait for the participant to finish")
    	input("Press Enter to continue...")
    	if query_yes_no("Start the clean up process?"):
    		subprocess.call(["./cleanup.sh"])	
    		print("\t\t You still need to terminate the auto_convert.sh script on cyb06")
    	experiment_counter += 1
    	print("\n\n\t ========= Next Experiment ========= \n\n")
    print("No experimental setups left. quitting ...")
