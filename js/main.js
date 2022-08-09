function make_row() {
	return main.append('div')
			   .style('width', '100%')
}

// media
media_path = './images/media.mp4'
overlay_text = 'Placing robotic manipulation within reach.'

var main = d3.select('#main')

var media_div = main.append('div')
				    .style('width', '100%')
				    .style('height', '100%')
				    .style('background', 'yellow')
					.style('position', 'relative')
				    .style('padding', '0')
var vid = media_div.append('video')
		 .attr('autoplay', 'autoplay')
		 .attr('loop', 'loop')
		 .attr('muted', 'muted')
		 .attr('playsinline', '')
		 .attr('width', '640')
		 .attr('height', '480')
		 .attr('filter', 'brightness(40%)')
		 .style('height', '100%')
		 .style('min-width', '100%')
		 .style('top', '50%')
		 .style('left', '50%')

vid.append('source')
	.attr('src', media_path)
	.attr('type', 'video/mp4')
	.style('transform', 'translate(-50%,0%)')
	.style('transform', '-moz-translate(-50%,0%)')
	.style('transform', '-o-translate(-50%,0%)')
	.style('transform', '-ms-translate(-50%,0%)')

media_div.append('div')
		 .text(overlay_text)
		 .style('position', 'absolute')
		 .style('top', '50%')
		 .style('color', 'white')
		 .style('left', '50%')
		 .style('transform', 'translate(-80%,-50%)')
		 .style('display', 'flex')
		 .style('line-height', '1.3')
		 .style('font-size', '5vw')
		 .style('font-weight', '530')
		 .style('letter-spacing', '3px')


// announcement

// mission statement


// real-sim compatibility and cross platform support

//
//
