// Initialize and add the map
function initMap() {
  // The location of Uluru
  const uluru = { lat: 40.0, lng: 40.0 };
  var myLatlng = new google.maps.LatLng(-25.363882,131.044922);

  // The map, centered at Uluru
  const map = new google.maps.Map(document.getElementById("map"), {
    zoom: 13,
    center: myLatlng,
	mapTypeId: 'satellite'
  });
  // The marker, positioned at Uluru
  var marker = new google.maps.Marker({
    position: myLatlng,
    map: map,
    draggable:true,
    title:"Drag me!"
});

}

window.initMap = initMap;