function [flag_takeoff]=takeoff_judge(closed_list, open_list)
global Hmin trapcount

% Hmin: the lowest H before current iteration
% trapcount: the count of iterations that Hmin does not change

Htemp=Hmin; % set a temporary variable Htemp = Hmin
[Hmin,~]=min(closed_list(:,6));     % find new Hmin
dH=Htemp-Hmin;

if dH~=0    % Hmin become lower -> getting closer to the goal
    trapcount=0;    % refresh trapcount
else
    trapcount=trapcount+1;
end

if trapcount>500 || isempty(open_list)
    flag_takeoff=true;
else
    flag_takeoff=false;
end
% fprintf('%d\n',trapcount)