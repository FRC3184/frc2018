var BUFFER_SIZE = 3000;
var MARGIN = 30;

function Chart(jq, name) {
    console.log(jq);
    this.element = jq;
    this.context = jq.get(0).getContext("2d");
    this.data = [];
    this.minY = 0;
    this.maxY = 0;
    this.color = "#0000bb";
    this.draw = function () {
        var ctx = this.context;
        var real_height = this.element.height();
        var real_width = this.element.width();
        var height = real_height - 2 * MARGIN;
        var width = real_width - 2 * MARGIN;
        ctx.clearRect(0, 0, real_width, real_height);
        if (this.data.length === 0) {
            return;
        }

        var minX = this.data[0].x;
        var maxX = this.data[this.data.length - 1].x;
        var minY = this.minY;
        var maxY = this.maxY;
        var xSize = width / (maxX - minX);
        var ySize = height / (this.maxY - this.minY);

        function transformX(x) {
            return ((x - minX) * xSize) + MARGIN;
        }
        function transformY(y) {
            return height - ((y - minY) * ySize) + MARGIN;
        }

        ctx.beginPath();
        ctx.rect(MARGIN, MARGIN, width, height);
        ctx.strokeStyle = "#000";
        ctx.stroke();

        ctx.font = "20px Georgia";
        var last_pt = this.data[this.data.length - 1];
        ctx.fillText(name + " (" + last_pt.y + ")", MARGIN + 5, MARGIN - 5);

        ctx.font = "10px Georgia";
        var count = Math.round(real_width / 40);
        for (var i = 0; i < count; i++) {
            ctx.fillText("" + Math.round((minX + i * (maxX - minX) / count) * 10) / 10,
                MARGIN + i * width / count, MARGIN + height + 10)
        }

        count = Math.round(real_height / 30);
        var step = (maxY - minY) / (count - 1);
        for (var i = 0; i < count; i++) {
            ctx.fillText("" + Math.floor((minY + i * step) * 100) / 100, 5,
                real_height - (MARGIN + i * height / (count - 1)), MARGIN - 5)
        }

        var first = true;
        this.data.forEach(function (point) {
            var x = transformX(point.x);
            var y = transformY(point.y);
            //console.log(x + ", " + y);
            if (first) {
                ctx.moveTo(x, y);
                first = false;
            }
            else {
                ctx.lineTo(x, y);
            }
        }, this);
        ctx.strokeStyle = this.color;
        ctx.stroke();
    };
    this.add_point = function (point) {
        var y = point.y;
        if (y < this.minY) {
            this.minY = y;
        }
        if (y > this.maxY) {
            this.maxY = y;
        }
        this.data.push(point);

        if (BUFFER_SIZE < this.data.length) {
            this.drop_oldest();
        }
        //this.recalc_extrema();
        this.draw();
    };
    this.recalc_extrema = function () {
        this.maxY = -Number.MAX_VALUE;
        this.minY = Number.MAX_VALUE;
        this.data.forEach(function (point) {
            var y = point.y;
            if (y < this.minY) {
                this.minY = y;
            }
            if (y > this.maxY) {
                this.maxY = y;
            }
        }, this);
    };
    this.drop_oldest = function () {
        var oldest_point = this.data[0];
        this.data = this.data.slice(1);
        var y = oldest_point.y;
    };
}