function make_row(bg='white') {
	return main.append('div')
			   .style('width', '100%')
			   .style('height', '100%')
			   .style('background-color', bg)
}
function add_video(div, media_src, overlay_text='') {
	let media_div = div.append('div')
					    .style('position', 'relative')
						.style('padding', 0)
	media_div.append('video')
			 .attr('autoplay', 'autoplay')
			 .attr('loop', 'loop')
			 .attr('muted', 'muted')
			 .attr('playsinline', '')
			 .attr('width', '640')
			 .attr('height', '480')
			 .style('filter', 'brightness(40%)')
			 .style('height', '100%')
			 .style('min-width', '100%')
			 .style('position', 'relative')
			 .style('top', '50%')
			 .style('left', '50%')
			 .style('transform', 'translate(-50%, 0%)')
			 .style('-webkit-transform', 'translate(-50%, 0%)')
			 .style('-moz-transform', 'translate(-50%, 0%)')
			 .style('-o-transform', 'translate(-50%, 0%)')
			 .style('-ms-transform', 'translate(-50%, 0%)')
			 .style('transform', 'translate(-50%, 0%)')
			 .append('source')
			 .attr('src', media_src)
			 .attr('type', 'video/mp4')

	media_div.append('div')
			 .style('position', 'absolute')
			 .style('top', '50%')
			 .style('left', '50%')
			 .style('transform', 'translate(-80%,-50%)')
			 .append('p')
			 .text(overlay_text)
			 .style('color', 'white')
			 .style('line-height', '1.3')
			 .style('font-size', '5vw')
			 .style('font-weight', '500')
			 .style('letter-spacing', '3px')
}
//////////////////////////////////////////////////////////////////////////

var main = d3.select('#main').style('font-family', 'Georgia,serif')

// media
media_path = './images/media.mp4'
overlay_text = 'Placing robotic manipulation within reach.'
add_video(make_row('red'), media_path, overlay_text)


// announcement
make_row('blue')

// mission statement


// real-sim compatibility and cross platform support

