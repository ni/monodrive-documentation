# Real-Time Ray Tracing Tuning

The maps available out of the box in monoDrive have been better tuned for Real-Time Ray Tracing. Real-Time Ray Tracing allows for accurate real-time reflections, better dynamic shadows, and ambient occlusion at the cost of frame rate on lower end machines. The primary viewport also defaults to ray tracing disabled but can be enabled by selecting the post processing volume in the map of interest and turning on ray traced ambient occlusion and ray traced reflections.

For better performance, the camera sensors default to `ray_tracing_enable: false`, but `ray_tracing_enable: true` can be set or added in the configuration at runtime to enable real time ray tracing. 

<p class="img_container">
    <img class='wide_img' src="../img/ray-tracing-off.png"/>
</p>


<p class="img_container">
    <img class='wide_img' src="../img/ray-tracing-on.png"/>
</p>