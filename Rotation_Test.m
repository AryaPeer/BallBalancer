function beam_sim_closed_form
%% === PUT YOUR NUMBERS (mm) HERE ===
l    = 85;    % half-beam length; total beam = 2*l
l11  = 45;    % crank DC
l12  = 105;   % connecting rod CB
w    = 60;    % horizontal offset from motor pivot D to beam pivot O
h    = 120;   % vertical offset from D to O

%% === "Same feel" as before, but now with hard end-stops ===
A11_deg = 50;       % requested amplitude (used as a speed scale only)
f_hz    = 0.6;      % nominal “feel” frequency for speed
T       = 8;        % seconds
dt      = 0.02;     % step
nsteps  = round(T/dt);

% We now integrate theta11 with a velocity and bounce at true limits
A11_rad = deg2rad(A11_deg);
% speed scale chosen so mid-stroke speed ~ sinusoid with same f, A
v_nom   = 2*pi*f_hz * A11_rad;   % rad/s
theta11 = 0.0;                   % start near center
v11     = +v_nom;                % initial direction

%% === Precompute constants for the closed-form solve ===
s     = hypot(w, h);         % sqrt(w^2 + h^2)
alpha = atan2(h, w);         % per slide
S     = (l12^2 - l11^2 - h^2 - w^2 - l^2) / s;

%% === Figure / axes ===
Lmax = max([2*l, w + l11 + l12, h + l11 + l12]);   % crude span
figure('Color','w'); hold on; grid on; axis equal;
xlabel('x (mm)'); ylabel('y (mm)'); title('Beam balancer — closed-form IK (hard end-stops)');
axis([-1.1*Lmax, 1.1*Lmax, -0.2*Lmax, 1.2*Lmax]);

% graphics
hBeam  = plot(nan, nan, 'k-', 'LineWidth', 3, 'DisplayName','Beam A–B');
hCrank = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName','Crank D–C');
hRod   = plot(nan, nan, 'r-', 'LineWidth', 2, 'DisplayName','Rod C–B');
plot(0,0,'ko','MarkerFaceColor','k','DisplayName','D (motor)');
hO     = plot(-w,h,'ks','MarkerFaceColor','y','DisplayName','O (pivot)');
legend('Location','northeastoutside');

%% === Sim loop (closed-form theta with true kinematic extremes) ===
theta_prev = 0;                 % continuity pick for the beam angle
for k = 1:nsteps
    % Trial integrate crank
    theta11_trial = theta11 + v11*dt;

    % Try closed-form at the trial angle
    [ok, theta, ~] = theta_closed_form(l, l11, l12, w, h, theta11_trial, s, alpha, S, theta_prev);

    if ok
        % Accept step
        theta11    = theta11_trial;
        theta_prev = theta;
    else
        % We attempted to cross the kinematic limit. Find the exact boundary
        % between (theta11) feasible and (theta11_trial) infeasible.
        theta11_edge = bisect_to_boundary(@(th) is_feasible(l, l11, l12, w, h, th, s, alpha, S), ...
                                          theta11, theta11_trial);

        % Snap to the boundary and compute beam angle exactly at the edge
        [~, theta_edge, ~] = theta_closed_form(l, l11, l12, w, h, theta11_edge, s, alpha, S, theta_prev);

        % Draw the edge frame, then reverse direction (hard end-stop “bounce”)
        theta11    = theta11_edge;
        theta_prev = theta_edge;
        v11        = -v11;
    end

    % ---- Forward geometry for drawing (at the accepted theta/theta11) ----
    D = [0, 0];
    O = [-w, h];
    C = [l11*cos(theta11),           l11*sin(theta11)];
    B = [-w + l*cos(theta_prev),   h + l*sin(theta_prev)];
    A = [-w - l*cos(theta_prev),   h - l*sin(theta_prev)];

    set(hBeam, 'XData',[A(1) B(1)], 'YData',[A(2) B(2)]);
    set(hCrank,'XData',[D(1) C(1)], 'YData',[D(2) C(2)]);
    set(hRod,  'XData',[C(1) B(1)], 'YData',[C(2) B(2)]);
    set(hO,    'XData',O(1), 'YData',O(2));
    drawnow;
end
end

% ==================== helpers ====================

function [ok, theta_pick, meta] = theta_closed_form(l, l11, l12, w, h, theta11, s, alpha, S, theta_prev)
% Closed-form from slides, rearranged: R*cos(theta - phi) + C = 0.
% Returns the solution closest to theta_prev (continuity).
A = -2*l*cos(alpha) - (2*l*l11/s)*cos(theta11);
B =  2*l*sin(alpha) - (2*l*l11/s)*sin(theta11);
C =  2*l11*cos(theta11 + alpha) - S;

R   = hypot(A, B);
phi = atan2(B, A);

meta.R = R; meta.phi = phi;

if R < 1e-12
    ok = false; theta_pick = NaN; return
end

x = -C / R;                          % desired cosine (must be in [-1, 1])
meta.x = x;

if abs(x) > 1 + 1e-10
    ok = false; theta_pick = NaN; return
end
x = max(min(x, 1), -1);              % numeric clamp

theta1 = phi + acos(x);
theta2 = phi - acos(x);

% pick the branch closest to the previous angle (wrap-aware)
d1 = angle_diff(theta1, theta_prev);
d2 = angle_diff(theta2, theta_prev);
theta_pick = theta1; if abs(d2) < abs(d1), theta_pick = theta2; end
ok = true;
end

function tf = is_feasible(l, l11, l12, w, h, theta11, s, alpha, S)
% Feasible iff the closed-form cosine falls within [-1, 1].
A = -2*l*cos(alpha) - (2*l*l11/s)*cos(theta11);
B =  2*l*sin(alpha) - (2*l*l11/s)*sin(theta11);
C =  2*l11*cos(theta11 + alpha) - S;
R = hypot(A, B);
if R < 1e-12
    tf = false; return
end
x = -C / R;
tf = (abs(x) <= 1 + 1e-10);
end

function root = bisect_to_boundary(isOK, a, b)
% Find the boundary between feasible (isOK=true) and infeasible (isOK=false)
% on the interval [a, b]. Assumes isOK(a)=true, isOK(b)=false (or vice versa).
fa = isOK(a); fb = isOK(b);
if fa == fb
    % If both same, shrink towards a boundary by small steps
    for i = 1:40
        mid = (a+b)/2;
        fm  = isOK(mid);
        if fm ~= fa
            a = (fa) * a + (~fa) * b; % just to satisfy lint; not used
        end
    end
    root = a; return
end
lo = a; hi = b;
for it = 1:50
    mid = 0.5*(lo+hi);
    fm  = isOK(mid);
    if fm == fa
        lo = mid; fa = fm;
    else
        hi = mid; fb = fm;
    end
    if abs(hi-lo) < 1e-8
        break
    end
end
root = 0.5*(lo+hi);
end

function d = angle_diff(a, b)
% shortest signed difference between angles (radians)
d = atan2(sin(a-b), cos(a-b));
end