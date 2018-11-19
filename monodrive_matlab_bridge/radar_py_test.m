if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end
mod = py.importlib.import_module('radar_bridge');

number_of_episodes = 1;
temp = py.mod.start(number_of_episodes)
%N = py.list({'Jones','Johnson','James'})

%names = py.mymod.search(1)
pause(10)
%py.monodrive_matlab_bridge.radar()