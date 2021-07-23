project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

incdir = include_directories('include')

executable('jetson_nav', 'main.cpp', 'stateMachine.cpp', 'rover.cpp', 'obstacle_avoidance/obstacleAvoidanceStateMachine.cpp', 'obstacle_avoidance/simpleAvoidance.cpp', 'pid.cpp', 'utilities.cpp',
			'search/spiralInSearch.cpp', 'search/lawnMowerSearch.cpp', 'search/searchStateMachine.cpp', 'search/spiralOutSearch.cpp',
            'gate_search/gateStateMachine.cpp', 'gate_search/diamondGateSearch.cpp',


            'test_nodes.cpp',



            'src/blackboard.cpp',
            'src/behavior_tree.cpp',
            'src/action_node.cpp',
            'src/tree_node.cpp',
            'src/shared_library.cpp',
            'src/shared_library_UNIX.cpp',
            'src/basic_types.cpp',
            'src/decorator_node.cpp',
            'src/control_node.cpp',
            'src/xml_parsing.cpp',
            'src/condition_node.cpp',
            'src/bt_factory.cpp',
            'src/loggers/bt_file_logger.cpp',
            
            'src/loggers/bt_cout_logger.cpp',
            'src/decorators/subtree_node.cpp',
            'src/decorators/inverter_node.cpp',
            'src/decorators/repeat_node.cpp',
            'src/decorators/delay_node.cpp',
            'src/decorators/retry_node.cpp',
            'src/controls/if_then_else_node.cpp',
            'src/controls/switch_node.cpp',
            'src/controls/reactive_fallback.cpp',
            'src/controls/sequence_star_node.cpp',
            'src/controls/while_do_else_node.cpp',
            'src/controls/fallback_node.cpp',
            'src/controls/parallel_node.cpp',
            'src/controls/sequence_node.cpp',
            'src/controls/reactive_sequence.cpp',
            'src/private/tinyxml2.cpp',
            'src/private/tinyxml2.h',
           dependencies : [liblcm],
           link_args : '-ldl',
           include_directories : incdir,
           install : true)
