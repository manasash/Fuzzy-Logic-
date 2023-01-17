function [T,E] = go( msg,path )
    global position N
    timeline=zeros(1,N);
    Energy=zeros(1,N);
    passtime=msg(2)/1000;
    pt=passtime
    for i=1:numel(path)
        timeline(path(i))=pt;
        pt=pt+passtime;
        if i<numel(path)
            Energy(path(i))=100*msg(2)*1e-6+100e-9*msg(2)*dist(position(:,i)',position(:,i+1))^2;
        else
            Energy(path(i))=50e-6*msg(2);
        end
    end
    T=timeline;
    E=Energy;

end