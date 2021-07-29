ISR_CODE void stepper_driver_interrupt_handler (void)
{
#ifdef ENABLE_BACKLASH_COMPENSATION
    static bool backlash_motion;
#endif

    // Start a step pulse when there is a block to execute.
    if(st->exec_block) {

        hal.stepper.pulse_start(st);

        st->new_block = st->dir_change = false;

        if (st->step_count == 0) // Segment is complete. Discard current segment.
            st->exec_segment = NULL;
    }

    // If there is no step segment, attempt to pop one from the stepper buffer
    if (st->exec_segment == NULL) {
        // Anything in the buffer? If so, load and initialize next step segment.
        if (segment_buffer_tail != segment_buffer_head) {

            // Initialize new step segment and load number of steps to execute
            st->exec_segment = (segment_t *)segment_buffer_tail;

            // Initialize step segment timing per step and load number of steps to execute.
            hal.stepper.cycles_per_tick(st->exec_segment->cycles_per_tick);
            st->step_count = st->exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.

            // If the new segment starts a new planner block, initialize stepper variables and counters.
            if (st->exec_block != st->exec_segment->exec_block) {

                if((st->dir_change = st->exec_block == NULL || st->dir_outbits.value != st->exec_segment->exec_block->direction_bits.value))
                    st->dir_outbits = st->exec_segment->exec_block->direction_bits;
                st->exec_block = st->exec_segment->exec_block;
                st->step_event_count = st->exec_block->step_event_count;
                st->new_block = true;
#ifdef ENABLE_BACKLASH_COMPENSATION
                backlash_motion = st->exec_block->backlash_motion;
#endif

                if(st->exec_block->overrides.sync)
                    sys.override.control = st->exec_block->overrides;

                // Execute output commands to be syncronized with motion
                while(st->exec_block->output_commands) {
                    output_command_t *cmd = st->exec_block->output_commands;
                    cmd->is_executed = true;
                    if(cmd->is_digital)
                        hal.port.digital_out(cmd->port, cmd->value != 0.0f);
                    else
                        hal.port.analog_out(cmd->port, cmd->value);
                    st->exec_block->output_commands = cmd->next;
                }

                // Enqueue any message to be printed (by foreground process)
                if(st->exec_block->message) {
                    if(message == NULL) {
                        message = st->exec_block->message;
                        protocol_enqueue_rt_command(output_message);
                    } else
                        free(st->exec_block->message); //
                    st->exec_block->message = NULL;
                }

                // Initialize Bresenham line and distance counters
                st->counter_x = st->counter_y = st->counter_z
                #ifdef A_AXIS
                = st->counter_a
                #endif
                #ifdef B_AXIS
                = st->counter_b
                #endif
                #ifdef C_AXIS
                = st->counter_c
                #endif
                #ifdef U_AXIS
                = st->counter_u
                #endif
                #ifdef V_AXIS
                = st->counter_v
                #endif
                = st->step_event_count >> 1;

            #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                memcpy(st->steps, st->exec_block->steps, sizeof(st->steps));
            #endif
            }

        #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
            // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
            st->amass_level = st->exec_segment->amass_level;
            st->steps[X_AXIS] = st->exec_block->steps[X_AXIS] >> st->amass_level;
            st->steps[Y_AXIS] = st->exec_block->steps[Y_AXIS] >> st->amass_level;
            st->steps[Z_AXIS] = st->exec_block->steps[Z_AXIS] >> st->amass_level;
        #ifdef A_AXIS
            st->steps[A_AXIS] = st->exec_block->steps[A_AXIS] >> st->amass_level;
        #endif
        #ifdef B_AXIS
            st->steps[B_AXIS] = st->exec_block->steps[B_AXIS] >> st->amass_level;
        #endif
        #ifdef C_AXIS
            st->steps[C_AXIS] = st->exec_block->steps[C_AXIS] >> st->amass_level;
        #endif
        #ifdef U_AXIS
            st->steps[U_AXIS] = st->exec_block->steps[U_AXIS] >> st->amass_level;
        #endif
        #ifdef V_AXIS
            st->steps[V_AXIS] = st->exec_block->steps[V_AXIS] >> st->amass_level;
        #endif
        #endif

            if(st->exec_segment->update_rpm) {
            #ifdef SPINDLE_PWM_DIRECT
                hal.spindle.update_pwm(st->exec_segment->spindle_pwm);
            #else
                hal.spindle.update_rpm(st->exec_segment->spindle_rpm);
            #endif
            }
        } else {
            // Segment buffer empty. Shutdown.
            st_go_idle();
            // Ensure pwm is set properly upon completion of rate-controlled motion.
            if (st->exec_block->dynamic_rpm && settings.mode == Mode_Laser)
                hal.spindle.set_state((spindle_state_t){0}, 0.0f);

            st->exec_block = NULL;
            system_set_exec_state_flag(EXEC_CYCLE_COMPLETE); // Flag main program for cycle complete

            return; // Nothing to do but exit.
        }
    }

    // Check probing state.
    // Monitors probe pin state and records the system position when detected.
    // NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
    if (sys.probing_state == Probing_Active && hal.probe.get_state().triggered) {
        sys.probing_state = Probing_Off;
        memcpy(sys.probe_position, sys.position, sizeof(sys.position));
        bit_true(sys.rt_exec_state, EXEC_MOTION_CANCEL);
    }

    // Execute step displacement profile by Bresenham line algorithm

    register axes_signals_t step_outbits = (axes_signals_t){0};

    st->counter_x += st->steps[X_AXIS];
    if (st->counter_x > st->step_event_count) {
        step_outbits.x = On;
        st->counter_x -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[X_AXIS] = sys.position[X_AXIS] + (st->dir_outbits.x ? -1 : 1);
    }

    st->counter_y += st->steps[Y_AXIS];
    if (st->counter_y > st->step_event_count) {
        step_outbits.y = On;
        st->counter_y -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[Y_AXIS] = sys.position[Y_AXIS] + (st->dir_outbits.y ? -1 : 1);
    }

    st->counter_z += st->steps[Z_AXIS];
    if (st->counter_z > st->step_event_count) {
        step_outbits.z = On;
        st->counter_z -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[Z_AXIS] = sys.position[Z_AXIS] + (st->dir_outbits.z ? -1 : 1);
    }

#ifdef A_AXIS
    st->counter_a += st->steps[A_AXIS];
    if (st->counter_a > st->step_event_count) {
        step_outbits.a = On;
        st->counter_a -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[A_AXIS] = sys.position[A_AXIS] + (st->dir_outbits.a ? -1 : 1);
    }
#endif

#ifdef B_AXIS
    st->counter_b += st->steps[B_AXIS];
    if (st->counter_b > st->step_event_count) {
        step_outbits.b = On;
        st->counter_b -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[B_AXIS] = sys.position[B_AXIS] + (st->dir_outbits.b ? -1 : 1);
    }
#endif

#ifdef C_AXIS
    st->counter_c += st->steps[C_AXIS];
    if (st->counter_c > st->step_event_count) {
        step_outbits.c = On;
        st->counter_c -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[C_AXIS] = sys.position[C_AXIS] + (st->dir_outbits.c ? -1 : 1);
    }
#endif

#ifdef U_AXIS
    st->counter_u += st->steps[U_AXIS];
    if (st->counter_u > st->step_event_count) {
        step_outbits.u = On;
        st->counter_u -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
    if(!backlash_motion)
#endif
            sys.position[U_AXIS] = sys.position[U_AXIS] + (st->dir_outbits.u ? -1 : 1);
    }
#endif

#ifdef V_AXIS
    st->counter_v += st->steps[V_AXIS];
    if (st->counter_v > st->step_event_count) {
        step_outbits.v = On;
        st->counter_v -= st->step_event_count;
#ifdef ENABLE_BACKLASH_COMPENSATION
    if(!backlash_motion)
#endif
            sys.position[V_AXIS] = sys.position[V_AXIS] + (st->dir_outbits.v ? -1 : 1);
    }
#endif

    st->step_outbits.value = step_outbits.value;

    // During a homing cycle, lock out and prevent desired axes from moving.
    if (state_get() == STATE_HOMING)
        st->step_outbits.value &= sys.homing_axis_lock.mask;

    // If segment is complete. Advance segment tail pointer.
    if (st->step_count == 0 || --st->step_count == 0) {
        segment_buffer_tail = segment_buffer_tail->next;
    }
}
