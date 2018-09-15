package com.argobot.argobottracker

import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView
import com.google.android.gms.location.*
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase

class MainActivity : AppCompatActivity() {

    private lateinit var locationCallback: LocationCallback
    private lateinit var fusedLocationClient: FusedLocationProviderClient

    private lateinit var mDatabase: DatabaseReference

    private val locationRequest = LocationRequest().apply {
        interval = 1000
        fastestInterval = 500
        priority = LocationRequest.PRIORITY_HIGH_ACCURACY
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        mDatabase = FirebaseDatabase.getInstance().reference;

        fusedLocationClient = LocationServices.getFusedLocationProviderClient(this)

        locationCallback = object : LocationCallback() {
            override fun onLocationResult(locationResult: LocationResult?) {
                locationResult ?: return
                for (location in locationResult.locations) {
                    val localLocation = ArgoLoc(location.latitude, location.longitude, location.speed)
                    mDatabase.child("locations").child("0").setValue(localLocation)
                    val text = "Lat: " + location.latitude + " Lng: " + location.longitude
                    findViewById<TextView>(R.id.location).text = text
                    findViewById<TextView>(R.id.speed).text = location.speed.toString()
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        startLocationUpdates()
    }

    override fun onPause() {
        super.onPause()
        stopLocationUpdates()
    }

    private fun stopLocationUpdates() {
        fusedLocationClient.removeLocationUpdates(locationCallback)
    }

    private fun startLocationUpdates() {
        fusedLocationClient.requestLocationUpdates(locationRequest,
                locationCallback,
                null /* Looper */)
    }
}
