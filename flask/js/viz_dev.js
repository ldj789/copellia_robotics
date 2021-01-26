let dat;

d3.json('/flask/data/output.json')
    .then(data => {
        dat = data;
        console.log(dat);
    });

