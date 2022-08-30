logo_file = "images/logo.svg"
logo_file_neg = "images/logo-negative.svg"
navigation_headers = [
		{text:'Platform', href: 'platform.html'},
		{text:'Software', href: "https://nuro-arm.readthedocs.io/en/latest/?"},
		{text:'Projects', href: 'projects.html'},
]
var header = d3.select("#header")
			   .style('background-color', 'white')
			   .append('div')
			   .style('width', "100%")
			   .style('max-width', "1260px")
			   .style('margin', "0 auto")
			   .style('height', "80px")
			   .style('display', "flex")
			   .style('justify-content', "space-between")

var logo_div = header.append('div').style('margin-top', 'auto').style('margin-bottom', 'auto')
					.append('a')
					.attr('href', './index.html')
					.append('div')
					.style('margin', "auto 10px")
					.style('display', 'flex')
logo_div.append('img')
	   .attr('src', logo_file)
	   .attr('alt', "logo")
	   .style('max-height', "60px")
	   .on('mouseover', function(d) {
		  d3.select(this).attr('src', logo_file_neg)
	   })
	   .on('mouseout', function(d) {
		  d3.select(this).attr('src', logo_file)
	   })
logo_div.append('p')
		.text('nuro.arm')
		.style('color', 'black')
		.style('margin', 'auto 0 auto 6px')
		.style('font-size', '2.4rem')
		.style('font-weight', '1000')
		.style('-webkit-transform', 'scale(1, 1.5)')
		.style('-moz-transform', 'scale(1, 1.5)')
		.style('-o-transform', 'scale(1, 1.5)')
		.style('transform', 'scale(1, 1.5)')

var nav = header.append('div')
			    .style('display', "flex")
			    .style('justify-content', "space-between")
				.style('margin', 'auto 0 15px 0')
for (let i=0; i<navigation_headers.length; i++) {
		nav.append('a')
			.attr('href', navigation_headers[i].href)
			.text(navigation_headers[i].text)
			.style('font-size', '1.2rem')
			.style('text-decoration', 'none')
			.style('text-transform', 'uppercase')
			.style('margin', '0 14px')
			.style('padding-bottom', '2px')
			.style('color', 'black')
			.on('mouseover', function(d) {
				d3.select(this).style('border-bottom', '1px solid black')
			})
			.on('mouseout', function(d) {
				d3.select(this).style('border-bottom', '1px solid #0000')
			})

}
