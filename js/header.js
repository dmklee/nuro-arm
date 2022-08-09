logo_file = "images/logo_text.svg"
logo_file_neg = "images/logo-negative_text.svg"
navigation_headers = [
	{text:'Platform', href: 'platform.html'},
	{text:'Software', href: "https://nuro-arm.readthedocs.io/en/latest/?"},
	{text:'Projects', href: 'projects.html'},
]
var header = d3.select("#header")
			   .append('div')
			   .style('width', "100%")
			   .style('max-width', "1440px")
			   .style('margin', "0 auto")
			   .style('height', "100px")
			   .style('display', "flex")
			   .style('justify-content', "space-between")

var logo = header.append('a')
				 .attr('href', './index.html')
				 .style('margin', "auto 10px")
				 .append('img')
				 .attr('src', logo_file)
				 .attr('alt', "logo")
				 .style('max-height', "60px")

var nav = header.append('div')
			    .style('display', "flex")
			    .style('justify-content', "space-between")
				.style('margin', 'auto 0 25px 0')
for (let i=0; i<navigation_headers.length; i++) {
	nav.append('a')
		.attr('href', navigation_headers[i].href)
		.text(navigation_headers[i].text)
		.style('font-size', '1.2rem')
		.style('text-decoration', 'none')
		.style('text-transform', 'uppercase')
		.style('margin', '0 20px')
		.style('padding-bottom', '2px')
		.style('color', 'black')

}
