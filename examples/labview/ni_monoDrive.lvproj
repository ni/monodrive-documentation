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
		<Item Name="Control 1.ctl" Type="VI" URL="../../../Control 1.ctl"/>
		<Item Name="Control 3.ctl" Type="VI" URL="../Control 3.ctl"/>
		<Item Name="Control 4.ctl" Type="VI" URL="../Control 4.ctl"/>
		<Item Name="Control 5.ctl" Type="VI" URL="../../../Control 5.ctl"/>
		<Item Name="Control 6.ctl" Type="VI" URL="../Control 6.ctl"/>
		<Item Name="Control 9.ctl" Type="VI" URL="../Control 9.ctl"/>
		<Item Name="mono_camera.vi" Type="VI" URL="../mono_camera.vi"/>
		<Item Name="mono_gps.vi" Type="VI" URL="../mono_gps.vi"/>
		<Item Name="mono_step.vi" Type="VI" URL="../mono_step.vi"/>
		<Item Name="ni_monoDrive.vi" Type="VI" URL="../ni_monoDrive.vi"/>
		<Item Name="Sensor Connections.ctl" Type="VI" URL="../Sensor Connections.ctl"/>
		<Item Name="Sensor Streams.ctl" Type="VI" URL="../Sensor Streams.ctl"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Draw Flattened Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Flattened Pixmap.vi"/>
				<Item Name="FixBadRect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/FixBadRect.vi"/>
				<Item Name="Flatten Pixmap.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pixmap.llb/Flatten Pixmap.vi"/>
				<Item Name="FormatTime String.vi" Type="VI" URL="/&lt;vilib&gt;/express/express execution control/ElapsedTimeBlock.llb/FormatTime String.vi"/>
				<Item Name="imagedata.ctl" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/imagedata.ctl"/>
				<Item Name="Is Value Changed.vim" Type="VI" URL="/&lt;vilib&gt;/Utility/Is Value Changed.vim"/>
				<Item Name="subElapsedTime.vi" Type="VI" URL="/&lt;vilib&gt;/express/express execution control/ElapsedTimeBlock.llb/subElapsedTime.vi"/>
			</Item>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
