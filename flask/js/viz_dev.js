/**
 * This looks good to get us started
 * https://observablehq.com/@sumitrk/scatter-plot
 */

let dat;
const width = 450,
    height = 450,
    margin = {
        left: 40, right: 20,
        top:20, bottom:40
    },
    pt_radius = 1,
    palette_colors = {
        gps: "#228C28",
        odometer: "#22288C",
        kf: "#227F8C"
    };

d3.json('/flask/data/output.json')
    .then(data => {

        let svg = d3.select("#plot");
        svg.attr("viewBox", [0, 0, width, height])
            .attr("height", height)
            .attr("width", width);

        const x = d3.scaleLinear()
            .domain([-5, 5])
            .rangeRound([margin.left, width - margin.right]);

        const y = d3.scaleLinear()
            .domain([-5, 5])
            .rangeRound([height - margin.bottom, margin.top]);

        const xAxis = svg
            .append('g')
            .call(d3.axisBottom(x))
            .attr("transform", `translate(0, ${height - margin.bottom})`);

        xAxis
            .append('text')
            .attr('font-family', 'sans-serif')
            .attr('font-size', 10)
            .attr('x', width / 2)
            .attr('y', margin.bottom - 5)
            .attr('text-anchor', 'end')
            .attr('fill', 'black')
            .attr('font-weight', 'bold')
            .text("x");

        const yAxis = svg
            .append('g')
            .attr("transform", `translate(${margin.left},0)`)
            .call(d3.axisLeft(y).ticks(5));

        yAxis
            .append('text')
            .attr('fill', 'black')
            .attr('font-family', 'sans-serif')
            .attr('font-size', 10)
            .attr('font-weight', 'bold')
            .attr('transform', 'rotate(-90)')
            .attr('y', -30)
            .attr('x', -height / 2)
            .text('y');

        svg
            .append('g')
            .selectAll('circle')
            .data(data)
            .join('circle')
            .attr('cx', d => x(d.gps_x))
            .attr('cy', d => y(d.gps_y))
            .attr('fill', palette_colors.gps)
            .attr('r', pt_radius);

        svg.append('g')
            .selectAll('circle')
            .data(data)
            .join('circle')
            .attr('cx', d => x(d.odometer_x))
            .attr('cy', d => y(d.odometer_y))
            .attr('fill', palette_colors.odometer)
            .attr('r', pt_radius);

        svg.append('g')
            .selectAll('circle')
            .data(data)
            .join('circle')
            .attr('cx', d => x(d.kf_x))
            .attr('cy', d => y(d.kf_y))
            .attr('fill', palette_colors.kf)
            .attr('r', pt_radius);

        dat = data;
        console.log(dat);
    });


