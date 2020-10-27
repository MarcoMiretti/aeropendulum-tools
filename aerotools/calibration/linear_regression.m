pkg load websockets
addpath ("../lib")

## utility function definitions
function ws = aero_poweron
  ws_connect ("192.168.100.41", "/start_pwm");
  pause (1);
  ws = ws_connect ("192.168.100.41", "/classic");
endfunction


function aero_set_duty (ws, propeller, duty_percent)
  # duty = duty_zero + duty_percent * c
  c = (propeller.duty_max - propeller.duty_zero) / 100;
  duty = dec2hex (floor (propeller.duty_zero + duty_percent * c));  
  message = cstrcat ('{"duty": "0x', duty, '"}');
  ws_send (ws, message, "text", 1);
endfunction


function [angle, err] = aero_read (ws, encoder)
  resp = ws_receive (ws);
  json = load_json (resp);
  decimal = hex2dec (json.angle(3:end));
  err = json.error
  if decimal > 65000
    decimal = 0;
  endif
  angle = 90 + (decimal - encoder.ninety_deg) * encoder.degree_per_pulse;
endfunction


## Sets duty to the PWM and keeps reading angle with a period of "step", when
## it stabilizes, it returns said angle.
function angle = set_duty_and_wait_until_steady (
    ws,
    aeropendulum,
    duty, tolerance,
    step
  )
  disp ("Looking for steady state angle for duty: "), disp (duty)
  stable_iterations = 0;
  prev_angle = 0;  # non reachable angle
  while stable_iterations < 4
    aero_set_duty(ws, aeropendulum.propeller, duty);
    [angle, err] = aero_read(ws, aeropendulum.encoder);
    if abs(angle - prev_angle) < tolerance
      stable_iterations++;
    elseif angle > 90  # unstable, abort
      disp ("Reaching unstable zone, abort")
      angle = -1;
      break
    else
      stable_iterations = 0;
    endif
    prev_angle = angle
    prev_err = err
    pause(step);
  endwhile
endfunction


function linear_regression(x, y)
  x_t = x';
  y_t = y';
  m = length(x_t);
  X = [ones(m, 1) x_t];
  theta = (pinv(X'*X))*X'*y_t;
  disp ("Calibration data:"), disp(theta)
  plot(x, y, "*", "markersize", 20)
  hold on
  plot(X(:,2), X*theta, "-")
  hold off
endfunction


## script starts here
aeropendulum.propeller.duty_on = 7210;
aeropendulum.propeller.duty_zero = 7530;
aeropendulum.propeller.duty_max = 10000;
aeropendulum.propeller.duty_total = 13107;
aeropendulum.encoder.ninety_deg = 1088;
aeropendulum.encoder.degree_per_pulse = 0.0625;


ws = aero_poweron()


## define useful constants
tolerance_deg = 0.3;
step_sec = 0.5;
first_index = 24;
steady_angles = zeros(1, 100)


## find steady angles for every duty point
for i = 18:100
  steady_angle = set_duty_and_wait_until_steady (
    ws,
    aeropendulum,
    i,
    tolerance_deg,
    step_sec
  );
  
  if steady_angle != -1
    steady_angles (i) = steady_angle;
  else
    last_index = i
    break
  endif
endfor


## set the duty to zero (so no one gets hurt)
aero_set_duty (ws, aeropendulum.propeller, 0)


## adjust the data
y_values = sin (deg2rad (steady_angles(first_index:last_index-1)));
x_values = linspace (first_index, last_index - 1, length (y_values));


## calculate linear regression
linear_regression (x_values, y_values)
xlabel ("PWM duty", "fontsize", 20)
ylabel ("sin(steady angle)", "fontsize", 20)
