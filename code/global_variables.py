def init():
    global complexity_level
    global controller_type
    global error
    global file_name_preASP_refined_domain
    global file_name_preASP_ToI_domain
    global file_name_preASP_abstract_domain
    global ASP_subfolder

    global file_name_preASP_ToI_planning
    global file_name_preASP_abstract_belief
    global file_name_preASP_refined_planning
    global file_name_preASP_inferring_indirect_observations
    global file_name_preASP_refined_world

    global abstract_sorts_string
    global abstract_attributes_string
    global refined_sorts_string
    global refined_attributes_string
    global refined_world_display_string
    global inferring_indirect_observations_display_string
    global testing_rules_string
    global planning_rules_string
    global max_number_steps_ToI_planning
    global refined_world_causal_law
    global new_refined_world_executability_condition
    global old_refined_world_executability_condition
    global sparc_path
    global results_file_name

    results_file_name = "simulation/results/"
    sparc_path = "$HOME/work/solverfiles/sparc.jar"

    ASP_subfolder = 'simulation/'

    abstract_sorts_string = ['#room = {library, kitchen}.\n#book = {book1}.',
                            '#room = {library, kitchen, office1}.\n#book = {book1, book2}.',
                            '#room = {library, kitchen, office1, office2}.\n#book = {book1, book2, book3}.',
                            '#room = {library, kitchen, office1, office2, storage_cupboard}.\n#book = {book1, book2, book3, book4}.']

    abstract_attributes_string = ['next_to(library, kitchen).',
                            'next_to(library, kitchen).\nnext_to(kitchen, office1).',
                            'next_to(library, kitchen).\nnext_to(kitchen, office1).\nnext_to(office1, office2).',
                            'next_to(library, kitchen).\nnext_to(kitchen, office1).\nnext_to(office1, office2).\nnext_to(office2, storage_cupboard).']

    refined_sorts_string = [('#coarse_place = {library,kitchen}.\n'
                            '#coarse_object = {book1}.\n'
                            '#object = {ref1_book1}.\n'
                            '#place = {c1, c2, c3, c4}.'),
                            ('#coarse_place = {library,kitchen,office1}.\n'
                            '#coarse_object = {book1,book2}.\n'
                            '#object = {ref1_book1,ref2_book1, ref1_book2,ref2_book2}.\n'
                            '#place = {c1, c2, c3, c4, c5, c6, c7, c8, c9}.\n'),
                            ('#coarse_place = {library,kitchen,office1,office2}.\n'
                            '#coarse_object = {book1,book2,book3}.\n'
                            '#object = {ref1_book1,ref2_book1,ref3_book1, ref1_book2,ref2_book2,ref3_book2, ref1_book3,ref2_book3,ref3_book3}.\n'
                            '#place = {c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16}.'),
                            ('#coarse_place = {library,kitchen,office1,office2,storage_cupboard}.\n'
                            '#coarse_object = {book1,book2,book3,book4}.\n'
                            '#object = {ref1_book1,ref2_book1,ref3_book1,ref4_book1, ref1_book2,ref2_book2,ref3_book2,ref4_book2, ref1_book3,ref2_book3,ref3_book3,ref4_book3, ref1_book4,ref2_book4,ref3_book4,ref4_book4}.\n\n'
                            '#place = {c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, c25}.')]
    refined_attributes_string = [('next_to(c1, c2).\nnext_to(c2, c3).\nnext_to(c3, c4).\n\n'
                                'comp(c1, library).\ncomp(c2, library).\ncomp(c3, kitchen).\ncomp(c4, kitchen).\n\n'
                                'comp(ref1_book1, book1).'),
                                ('next_to(c1, c2).\nnext_to(c2, c3).\nnext_to(c3, c4).\nnext_to(c4, c5).\nnext_to(c5, c6).\nnext_to(c6, c7).\nnext_to(c7, c8).\nnext_to(c8, c9).\n\n'
                                'comp(c1, library).\ncomp(c2, library).\ncomp(c3, library).\ncomp(c4, kitchen).\ncomp(c5, kitchen).\ncomp(c6, kitchen).\ncomp(c7, office1).\ncomp(c8, office1).\ncomp(c9, office1).\n\n'
                                'comp(ref1_book1, book1).\ncomp(ref2_book1, book1).\ncomp(ref1_book2, book2).\ncomp(ref2_book2, book2).'),
                                ('next_to(c1, c2).\nnext_to(c2, c3).\nnext_to(c3, c4).\nnext_to(c4, c5).\nnext_to(c5, c6).\nnext_to(c6, c7).\nnext_to(c7, c8).\nnext_to(c8, c9).\nnext_to(c9, c10).\nnext_to(c10, c11).\nnext_to(c11, c12).\nnext_to(c12, c13).\nnext_to(c13, c14).\nnext_to(c14, c15).\nnext_to(c15, c16).\n\n'
                                'comp(c1, library).\ncomp(c2, library).\ncomp(c3, library).\ncomp(c4, library).\ncomp(c5, kitchen).\ncomp(c6, kitchen).\ncomp(c7, kitchen).\ncomp(c8, kitchen).\ncomp(c9, office1).\ncomp(c10, office1).\ncomp(c11, office1).\ncomp(c12, office1).\ncomp(c13, office2).\ncomp(c14, office2).\ncomp(c15, office2).\ncomp(c16, office2).\n\n'
                                'comp(ref1_book1, book1).\ncomp(ref2_book1, book1).\ncomp(ref3_book1, book1).\ncomp(ref1_book2, book2).\ncomp(ref2_book2, book2).\ncomp(ref3_book2, book2).\ncomp(ref1_book3, book3).\ncomp(ref2_book3, book3).\ncomp(ref3_book3, book3).'),
                                ('next_to(c1, c2).\nnext_to(c2, c3).\nnext_to(c3, c4).\nnext_to(c4, c5).\nnext_to(c5, c6).\nnext_to(c6, c7).\nnext_to(c7, c8).\nnext_to(c8, c9).\nnext_to(c9, c10).\nnext_to(c10, c11).\nnext_to(c11, c12).\nnext_to(c12, c13).\nnext_to(c13, c14).\nnext_to(c14, c15).\nnext_to(c15, c16).\nnext_to(c16, c17).\nnext_to(c17, c18).\nnext_to(c18, c19).\nnext_to(c19, c20).\nnext_to(c20, c21).\nnext_to(c21, c22).\nnext_to(c22, c23).\nnext_to(c23, c24).\nnext_to(c24, c25).'
                                'comp(c1, library).\ncomp(c2, library).\ncomp(c3, library).\ncomp(c4, library).\ncomp(c5, library).\ncomp(c6, kitchen).\ncomp(c7, kitchen).\ncomp(c8, kitchen).\ncomp(c9, kitchen).\ncomp(c10, kitchen).\ncomp(c11, office1).\ncomp(c12, office1).\ncomp(c13, office1).\ncomp(c14, office1).\ncomp(c15, office1).\ncomp(c16, office2).\ncomp(c17, office2).\ncomp(c18, office2).\ncomp(c19, office2).\ncomp(c20, office2).\ncomp(c21, storage_cupboard).\ncomp(c22, storage_cupboard).\ncomp(c23, storage_cupboard).\ncomp(c24, storage_cupboard).\ncomp(c25, storage_cupboard).'
                                'comp(ref1_book1, book1).\ncomp(ref2_book1, book1).\ncomp(ref3_book1, book1).\ncomp(ref4_book1, book1).\ncomp(ref1_book2, book2).\ncomp(ref2_book2, book2).\ncomp(ref3_book2, book2).\ncomp(ref4_book2, book2).\ncomp(ref1_book3, book3).\ncomp(ref2_book3, book3).\ncomp(ref3_book3, book3).\ncomp(ref4_book3, book3).\ncomp(ref1_book4, book4).\ncomp(ref2_book4, book4).\ncomp(ref3_book4, book4).\ncomp(ref4_book4, book4).')]
    max_number_steps_ToI_planning = [3,6,8,10]

    testing_rules_string = ('% Make sure the outcome of any concrete action is tested\n'
                        'occurs(test(R, loc(R, C), true), I+1) :- occurs(move(R, C), I).\n'
                        'occurs(test(R, in_hand(R, O), true), I+1) :- occurs(pickup(R, O), I).\n'
                        'occurs(test(R, in_hand(R, O), false), I+1) :- occurs(put_down(R, O), I).\n'
                        '-occurs(pickup(rob1, OP), I) :- holds(loc(rob1, C), I), not occurs(test(rob1, loc(OP, C), true), I-1).\n'
                        '-occurs(pickup(rob1, OP), I) :- I = 0.')


    planning_rules_string = ('%% Failure is not an option.\n'
                            'success :- goal(I).\n'
                            ':- not success.\n'
                            '%% Plan Actions minimally\n'
                            'occurs(A,I):+ not goal(I).\n'
                            '%% Preventing preASP_refined_domain_no_planning\n'
                            'something_happened(I) :- occurs(A, I).\n'
                            ':- not goal(I), not something_happened(I).\n'
                            ':- not something_happened(0).')

    inferring_indirect_observations_display_string = 'holds(indirectly_observed(rob1,B,C),numSteps).'

    refined_world_display_string = 'holds(loc(A,B),numSteps).\nholds(in_hand(A,B),numSteps).\nholds(coarse_loc(A,B),numSteps).\nholds(coarse_in_hand(A,B),numSteps).\n'
    refined_world_causal_law = '-holds(in_hand(R,OP2),I+1) :- occurs(put_down(rob1,OP1),I), comp(OP1,B), comp(OP2,B), holds(coarse_in_hand(rob1,B),I).'
    new_refined_world_executability_condition = '-occurs(put_down(R,OP),I) :- comp(OP,B), -holds(coarse_in_hand(rob1,B),I).'
    old_refined_world_executability_condition = '-occurs(put_down(R, OP), I) :-  -holds(in_hand(R, OP), I).'

'''    testing_rules_string = ('% Make sure the outcome of any concrete action is tested\n'
                        'occurs(test(rob1, F, true), I) :- -holds(F, I-1), holds(F, I), #physical_inertial_fluent(F).\n'
                        'occurs(test(rob1, F, false), I) :- holds(F, I-1), -holds(F, I), #physical_inertial_fluent(F), not -occurs(test(rob1, F, false), I).\n'
                        '-occurs(test(R, F, O), 0). % cannot test in the first step\n',
                        '-occurs(pickup(rob1, OP), I) :- holds(loc(rob1, C), I), not occurs(test(rob1, loc(OP, C), true), I-1).\n'
                        '-occurs(pickup(rob1, OP), I) :- I = 0.)'
'''
