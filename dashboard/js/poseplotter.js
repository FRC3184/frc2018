var last_time = Date.now();
var robot_pose = {x: 0, y: 0, heading: 0};
var robot_width = 2;
var old_poses = [robot_pose];
var field_img = document.getElementById("field-img");
var arbitrary_lines = [];

source.addEventListener('draw', function(event) {
    var data = JSON.parse(event.data);
    arbitrary_lines.push(data.points);
});

source.addEventListener('pose', function(event) {{

    var delta = Date.now() - last_time;
    last_time += delta;

    var data = JSON.parse(event.data);
    robot_pose = data;
    old_poses = old_poses.concat(robot_pose);


    var element = $("#simulbot");
    var canvas = element.get(0).getContext("2d");
    var h = element.height();
    var w = element.width();
    var SCALE = 1;
    var r_w = robot_width * 12;
    var r_h = 60 - r_w;
    r_w *= SCALE;
    r_h *= SCALE;

    canvas.clearRect(0, 0, w, h);

    // Gridlines
    canvas.beginPath();
    canvas.save();
    canvas.setTransform(1, 0, 0, 1, 0, 0);
    canvas.strokeStyle = "#000";
    canvas.lineWidth = 1;
    canvas.translate(w/2 -robot_pose.x, h/2 -robot_pose.y);
    // Field image
    field_scale = 12/15;
    canvas.drawImage(field_img, 0, -202 * field_scale, 397 * field_scale, 404 * field_scale);

    var count = 50;
    var step = 12;

    for (var i = 0; i < 2 * count + 1; i++) {
        canvas.moveTo(-count * step, -count * step + i * step);
        canvas.lineTo(count * step, -count * step + i * step);
    }
    for (var i = 0; i < 2 * count + 1; i++) {
        canvas.moveTo(-count * step + i * step, -count * step);
        canvas.lineTo(-count * step + i * step, count * step);
    }

    canvas.stroke();

    // Draw axes
    canvas.beginPath();
    canvas.lineWidth = 4;
    canvas.moveTo(-count * step, 0);
    canvas.lineTo(count * step, 0);
    canvas.moveTo(0, -count * step);
    canvas.lineTo(0, count * step);
    canvas.stroke();

    canvas.restore();

    // Arbitrary stuff
    canvas.save();
    canvas.setTransform(1, 0, 0, 1, 0, 0);

    canvas.strokeStyle = "#ff0000";
    canvas.lineWidth = 2;
    for (var j=0; j < arbitrary_lines.length; j++) {
        var pts = arbitrary_lines[j];
        canvas.translate(w/2 -robot_pose.x, h/2 -robot_pose.y);

        canvas.beginPath();
        canvas.moveTo(pts[0].x, pts[0].y);
        for (var i = 0; i < pts.length-1; i++) {
            var pt = pts[i+1];
            canvas.lineTo(pt.x, pt.y);
        }
        canvas.stroke();
        canvas.setTransform(1, 0, 0, 1, 0, 0);
    }
    canvas.restore();


    // Draw pose history
    canvas.beginPath();

    canvas.save();
    canvas.setTransform(1, 0, 0, 1, 0, 0);
    canvas.translate(w/2 - robot_pose.x, h/2 - robot_pose.y);
    canvas.strokeStyle = "#0000FF";
    canvas.lineWidth = 3;
    var first_pose = old_poses[0];
    canvas.moveTo(first_pose.x, first_pose.y);
    for (var i = 1; i < old_poses.length; i++) {

        var next_pose = old_poses[i];
        canvas.lineTo(next_pose.x, next_pose.y);
    }
    canvas.stroke();
    canvas.restore();

    // Robot
    canvas.beginPath();
    canvas.save();
    canvas.setTransform(1, 0, 0, 1, 0, 0);
    canvas.strokeStyle = "#FF0000";
    canvas.lineWidth = 4;
    canvas.translate(w/2, h/2);
    canvas.rotate(-robot_pose.heading + Math.PI / 2);

    canvas.rect(-r_w/2, -r_h/2, r_w, r_h);
    canvas.moveTo(0, 0);
    canvas.lineTo(0, -r_h/2);
    canvas.stroke();
    canvas.restore();


}}, false);