package org.firstinspires.ftc.teamcode.toolkit

import android.media.MediaPlayer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.R

class MusicPlayer (hardwareMap: HardwareMap, option1 : Boolean) {
    private val mediaPlayer: MediaPlayer =
            if (option1) MediaPlayer.create(hardwareMap.appContext, R.raw.angrybirds)
            else MediaPlayer.create(hardwareMap.appContext, R.raw.badpiggies)

    init {
        mediaPlayer.isLooping = true
        mediaPlayer.seekTo(0)
    }

    fun play() {
        if (!mediaPlayer.isPlaying) mediaPlayer.start()
    }
    fun pause() {
        mediaPlayer.pause()
    }
    fun stop() {
        mediaPlayer.stop()
        mediaPlayer.release()
    }
    fun isPlaying() = mediaPlayer.isPlaying
}