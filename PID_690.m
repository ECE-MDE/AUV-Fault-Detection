classdef PID_690
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        kp
        kd
        ki
        i_error_sum=0.0;
        %boolean value: true if controlled parameter is an angle, 
        % false otherwise 
        units_RAD=true; 
        Ts=0.1 % sample time equvialent to 1 Hz sample frequency
        i_output_min
        i_output_max
        output_min
        output_max

        %type: pitch or depth=1,yaw=2
        type
    end

    methods
        function obj = PID_690(p_gain,i_gain,d_gain,units_RAD,type,Ts,int_max,out_max)
            
            obj.kp= p_gain;
            obj.kd= d_gain;
            obj.ki=i_gain;
            obj.units_RAD=units_RAD;
            obj.type = type;
            obj.Ts=Ts;
            obj.i_output_min=-abs(int_max);
            obj.i_output_max=abs(int_max);
            obj.output_min=-abs(out_max);
            obj.output_max=abs(out_max);
        end

        function [output,obj,setpoint] = control_loop(obj,input,rate,setpoint)
           
            if obj.type==2
                setpoint=obj.course_norm(setpoint,input);
            end
            error=setpoint-input;

            
            p_error=error;
            i_error=error*obj.Ts;
            d_error=-rate;
            new_i_error_sum=obj.i_error_sum+i_error;

            p_output=obj.kp*p_error;
            d_output=obj.kd*d_error;
            i_output=obj.ki*new_i_error_sum;

            if i_output>obj.i_output_max
                i_output=obj.i_output_max;
                i_saturated=true;
            elseif i_output<obj.i_output_min
                i_output=obj.i_output_min;
                i_saturated=true;
            else
                i_saturated=false;
            end
            output=p_output+d_output+i_output;

            if output>obj.output_max
                output=obj.output_max;
                output_saturated=true;
            elseif output<obj.output_min
                output=obj.output_min;
                output_saturated=true;
            else
                output_saturated=false;
            end
            
            if ~output_saturated && ~i_saturated && obj.ki~=0.0
                obj.i_error_sum=new_i_error_sum;
            end

        end


        function cmd=course_norm(obj,command,crs)
            command=mod(command,2*pi);
            [min_turn,turn_dir]=min([mod(command-crs,2*pi),mod(crs-command,2*pi)]);
            if turn_dir==1 %turn to STBD
                cmd=crs+min_turn;
            else %turn to port
                cmd=crs-min_turn;
            end

        end


       
    end
end