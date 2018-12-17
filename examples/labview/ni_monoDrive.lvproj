<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="18008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="SubVIs" Type="Folder"/>
		<Item Name="mono_bbox.vi" Type="VI" URL="../mono_bbox.vi"/>
		<Item Name="mono_camera.vi" Type="VI" URL="../mono_camera.vi"/>
		<Item Name="mono_camera_test.vi" Type="VI" URL="../mono_camera_test.vi"/>
		<Item Name="mono_commands.ctl" Type="VI" URL="../mono_commands.ctl"/>
		<Item Name="mono_config_parser_example.vi" Type="VI" URL="../mono_config_parser_example.vi"/>
		<Item Name="mono_connect.vi" Type="VI" URL="../mono_connect.vi"/>
		<Item Name="mono_gps.vi" Type="VI" URL="../mono_gps.vi"/>
		<Item Name="mono_header.vi" Type="VI" URL="../mono_header.vi"/>
		<Item Name="mono_imu.vi" Type="VI" URL="../mono_imu.vi"/>
		<Item Name="mono_radar.vi" Type="VI" URL="../mono_radar.vi"/>
		<Item Name="mono_rpm.vi" Type="VI" URL="../mono_rpm.vi"/>
		<Item Name="mono_send_camera_config.vi" Type="VI" URL="../mono_send_camera_config.vi"/>
		<Item Name="mono_send_command.vi" Type="VI" URL="../mono_send_command.vi"/>
		<Item Name="mono_send_simulator_config.vi" Type="VI" URL="../mono_send_simulator_config.vi"/>
		<Item Name="mono_send_vehicle_command.vi" Type="VI" URL="../mono_send_vehicle_command.vi"/>
		<Item Name="mono_send_vehicle_config.vi" Type="VI" URL="../mono_send_vehicle_config.vi"/>
		<Item Name="mono_sensor_init.vi" Type="VI" URL="../mono_sensor_init.vi"/>
		<Item Name="mono_simulator.vi" Type="VI" URL="../mono_simulator.vi"/>
		<Item Name="mono_step.vi" Type="VI" URL="../mono_step.vi"/>
		<Item Name="mono_test.ctl" Type="VI" URL="../mono_test.ctl"/>
		<Item Name="ni_monoDrive.vi" Type="VI" URL="../ni_monoDrive.vi"/>
		<Item Name="python_test.vi" Type="VI" URL="../python_test.vi"/>
		<Item Name="Sensor Connections.ctl" Type="VI" URL="../Sensor Connections.ctl"/>
		<Item Name="Sensor Streams.ctl" Type="VI" URL="../Sensor Streams.ctl"/>
		<Item Name="Sensors.ctl" Type="VI" URL="../Sensors.ctl"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Draw Flattened Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Flattened Pixmap.vi"/>
				<Item Name="FixBadRect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/FixBadRect.vi"/>
				<Item Name="Flatten Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pixmap.llb/Flatten Pixmap.vi"/>
				<Item Name="FormatTime String.vi" Type="VI" URL="/&lt;vilib&gt;/express/express execution control/ElapsedTimeBlock.llb/FormatTime String.vi"/>
				<Item Name="imagedata.ctl" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/imagedata.ctl"/>
				<Item Name="Is Value Changed.vim" Type="VI" URL="/&lt;vilib&gt;/Utility/Is Value Changed.vim"/>
				<Item Name="RGB to Color.vi" Type="VI" URL="/&lt;vilib&gt;/Utility/colorconv.llb/RGB to Color.vi"/>
				<Item Name="subElapsedTime.vi" Type="VI" URL="/&lt;vilib&gt;/express/express execution control/ElapsedTimeBlock.llb/subElapsedTime.vi"/>
			</Item>
			<Item Name="mono_config_lidar.ctl" Type="VI" URL="../mono_config_lidar.ctl"/>
			<Item Name="mono_config_vehicle.ctl" Type="VI" URL="../mono_config_vehicle.ctl"/>
			<Item Name="Simulator_Configuration.ctl" Type="VI" URL="../Simulator_Configuration.ctl"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
