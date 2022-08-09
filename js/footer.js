bg_color = "#0d0d0d"
socials=[
	{name:'Github', img:'images/github-logo.png', href:'https://github.com/dmklee/nuro-arm'},
	{name:'Email', img:'images/email-logo.png', href:'mailto:klee.d@northeastern.edu'},
];
footer_text = "© 2022 by David Klee."

var footer = d3.select("#footer")
			   .append('div')
			   .style('width', "100%")
			   .style('height', "80px")
			   .style('padding', "20px")
			   .style('background', bg_color)
			   .style('display', "float")

var socials_div = footer.append('div')
					    .style('display', "flex")
			   			.style('justify-content', "space-between")
			   			.style('width', "20%")
			   			.style('max-width', "200px")
			   			.style('margin', "0 auto")
for (let i=0; i<socials.length; i++) {
	socials_div.append('a')
				.attr('href',socials[i].href)
				.append('img')
				.attr('src', socials[i].img)
				.attr('alt', socials[i].name)
				.style('width','40px')
				.style('height','40px')
				.style('opacity','80%')
}

footer.append('p')
	  .text(footer_text)
	  .style('color', 'white')
	  .style('margin', '20px auto')
	  .style('display', 'flex')
	  .style('justify-content', 'center')
  	  .style('opacity','80%')
