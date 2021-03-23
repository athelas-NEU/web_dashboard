# Athelas Web Dashboard


# Requirements
- Bootstrap 4, Font awesome 4, Font Awesome 5 Free & Pro snippets 
- Live Server
- Nginx

### ROS (melodic) Packages
- rosbridge_server
- web_video_server

### Javascript modules
The following modules are pre-loaded with their CDN sources:
- moment.min.js
- ZingChart

There may be a few others that will require npm to install

# Setup
Use your favorite editor to update the file in the command below:

```
sudo vim /etc/nginx/sites-enabled/default
```

In the file, find the line 

```
root /var/www/html;
```

and replace `/var/www/html;` with the path to this repo. Nginx will find the index.html file in that folder.
For example:

```
root /home/unnas/athelas_webui;
```

Restart Nginx:
```
sudo systemctl restart nginx
```

To determine the IP address that the dashboard is now being hosted, use:
```
hostname -I
```

# Run

Launch the server with

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

If you are adding this to an existing launch file, you can use
```
	<include file="/opt/ros/melodic/share/rosbridge_server/launch/rosbridge_websocket.launch" />
```