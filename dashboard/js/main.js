var name_map = {};
var chooser_map = {};
var connect_state = false;
var my_grid = [100, 100];
var drag_opts = {
    grid: my_grid,
    stop: function (evt, ui) {
        ui.position = roundVector(ui.position, my_grid);
    }
};

function fitToContainer(canvas) {
    // Make it visually fill the positioned parent
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    // ...then set the internal size to match
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;
}

var resize_opts = {
    grid: my_grid, resize: function (event, ui) {
        var canvases = ui.element.children("canvas");
        if (canvases.length > 0) {
            fitToContainer(canvases.get(0));
        }
    }
};

/*
 * Rounds both parts of vec_k to nearest roundMagnitude_k
 * Returns rounded vector
*/
function roundVector(vec, round) {
    var ret = vec;
    ret.left = Math.round(vec.left / round[0]) * round[0];
    ret.top = Math.round(vec.top / round[1]) * round[1];
    return ret;
}

var make_chart = function (data) {
    console.log(data.name);
    var chart_element = $("<canvas width=\"400px\" height=\"400px\" class=\"chart\"></canvas>");
    chart_element.appendTo("#dashboard");
    chart_wrapper = $("<div class=\"element-wrap ui-widget-content\" data-name=\"" + data.name + "\"></div>");
    chart_element.wrap(chart_wrapper);

    $("#elements-toggle").append($("<div class='toggle-block'><label for='__toggle-" + data.name + "'>" + data.name + "</label><input type='checkbox' class='display-toggle' id='__toggle-" + data.name + "' data-target='" + data.name + "' checked /></div><br /> "));

    var my_chart = new Chart(chart_element, data.name);

    name_map[data.name] = my_chart;
};
var make_chooser = function (data) {
    var chart_element = $("<select class=\"chooser\" data-name=\"" + data.name + "\"></select>");
    chart_element.append("<option>&lt;Select&gt;</option>");
    data.options.forEach(function (item) {
        chart_element.append("<option value=\"" + item + "\">" + item + "</option>");
    });
    chart_element.appendTo("#dashboard");
    chart_wrapper = $("<div class=\"element-wrap ui-widget-content\" style=\"height:100px;width:100px\" data-name=\"" + data.name + "\"></div>");
    chart_wrapper.append($("<span>" + data.name + "</span><br />"));
    chart_element.wrap(chart_wrapper);

};
var make_indicator = function (data) {
    var element = $("<span>" + data.name + "</span>");
    element.appendTo("#dashboard");
    element.wrap($("<div class=\"indicator inactive element-wrap ui-widget-content\" style=\"height:100px;width:100px\" data-name=\"" + data.name + "\"></div>"));
};
var make_input = function (data) {
    var element = $("<input class=\"number-input\" type=\"number\" data-name=\"" + data.name + "\" value=\"" + data.value + "\"/>");
    element.appendTo("#dashboard");
    element.wrap($("<div class=\"element-wrap ui-widget-content\" style=\"height:100px;width:100px\" data-name=\"" + data.name + "\"></div>"));
    element.before("<label>" + data.name + "</label>")
};
var make_extension = function (data) {
    var element = $(data.html);
    element.appendTo("#dashboard");
    element.wrap($("<div class=\"element-wrap ui-widget-content\" data-name=\"" + data.name + "\"></div>"));
};

var source;
function initEventSource() {
    source = new EventSource("/events");
    source.onerror = function (e) {
        if (es.readyState === 2) {
            setTimeout(initEventSource, 1000);
        }
    }
}
initEventSource();


source.addEventListener('message', function (event) {
    var data = JSON.parse(event.data);
    alert("The server says " + data.message);
}, false);
source.addEventListener('data', function (event) {
    var data = JSON.parse(event.data);
    var name = data.name;
    var t = data.time;
    var y = data.value;

    if (name in name_map) {
        name_map[name].add_point({ x: t, y: y });
    }
});
source.addEventListener('indicator', function (event) {
    var data = JSON.parse(event.data);
    var name = data.name;
    var status = data.status;

    var element = $(".indicator[data-name='" + name + "']");
    if (element.is(".inactive") && status) {
        element.removeClass("inactive");
        element.addClass("active");
    }
    else if (element.is(".active") && !status) {
        element.removeClass("active");
        element.addClass("inactive");
    }
});
source.addEventListener("action", function (event) {
    var data = JSON.parse(event.data);
    if (data.name in name_map || data.name in chooser_map) {
        return;
    }
    switch (data.action) {
        case "make_extension":
            make_extension(data);
            break;
        case "make_chart":
            make_chart(data);
            break;
        case "make_chooser":
            make_chooser(data);
            break;
        case "make_indicator":
            make_indicator(data);
            break;
        case "make_input":
            make_input(data);
            break;
    }
});

$(document).ready(function () {

    $("#save-button").click(function () {
        $(".element-wrap").each(function (i, element) {
            element = $(element);  //Why jQuery doesn't return these as jQuery objects is beyond me.
            var name = element.data("name");
            var data = element.position();
            data.width = element.width();
            data.height = element.height();
            data.shown = !element.is(":hidden");
            localStorage.setItem(name, JSON.stringify(data));
        });
    });
    $("#refresh-button").click(function () {
        source.close();
        location.reload();
    });

    setTimeout(function () {
        $(".element-wrap").draggable(drag_opts).resizable(resize_opts).draggable("disable").resizable("disable");

        $(".chooser").change(function () {
            var datum = { name: $(this).data("name"), option: $(this).val() };
            $.post("/update_chooser", JSON.stringify(datum))
        });

        $("#editable-check").change(function () {
            $(".element-wrap").draggable(this.checked ? "enable" : "disable");
            $(".element-wrap").resizable(this.checked ? "enable" : "disable");
        });
        $(".display-toggle").click(function () {
            var name = $(this).data("target");

            var chart = name_map[name].element;
            var elem = chart.parent();
            if ($(this).is(":checked")) {
                elem.show({
                    complete: function () {

                        var stored_data = localStorage.getItem(name);
                        if (stored_data !== null) {
                            stored_data = JSON.parse(stored_data);
                            var css = {
                                width: stored_data.width,
                                height: stored_data.height,
                                left: stored_data.left,
                                top: stored_data.top
                            };
                            elem.css(css);
                        }
                        fitToContainer(chart.get(0));
                    }
                });
            }
            else {
                elem.hide();
            }
        });
        $(".number-input").change(function (evt) {
            var datum = { name: $(this).data("name"), value: $(this).val() };
            $.post("/update_number", JSON.stringify(datum))
        });

        console.log("Loading positions...");
        $(".element-wrap").each(function (i, element) {
            element = $(element);  //Why jQuery doesn't return these as jQuery objects is beyond me.
            var name = element.data("name");
            var stored_data = localStorage.getItem(name);
            if (stored_data !== null) {
                stored_data = JSON.parse(stored_data);
                var css = {
                    width: stored_data.width,
                    height: stored_data.height,
                    left: stored_data.left,
                    top: stored_data.top
                };
                element.css(css);
                if (!stored_data.shown) {
                    element.hide();
                    $("input[data-target='" + name + "']").prop("checked", false);
                }
            }
            if (element.children("canvas").length > 0) {
                fitToContainer(element.children("canvas").get(0));
            }
        });
        console.log("Loaded positions");
    }, 500);

    setInterval(function () {
        var elem = $("#robot-status");
        switch (source.readyState) {
            case 0:
                elem.removeClass("connected");
                elem.removeClass("closed");
                elem.addClass("connecting");
                elem.html("Connecting...");
                if (connect_state) {
                    connect_state = false;
                }
                break;
            case 1:
                elem.addClass("connected");
                elem.removeClass("closed");
                elem.removeClass("connecting");
                elem.html("Connected!");
                if (!connect_state) {
                    $(".chooser").change();
                    $(".number-input").change();
                    connect_state = true;
                }
                break;
            case 2:
                elem.removeClass("connected");
                elem.addClass("closed");
                elem.removeClass("connecting");
                elem.html("Not connected!");
                if (connect_state) {
                    connect_state = false;
                }
                break;
        }
    }, 500);
});
