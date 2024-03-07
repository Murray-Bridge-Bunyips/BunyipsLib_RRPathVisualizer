/**
 * Utility to convert between inches and metric units.
 * [src](https://github.com/Murray-Bridge-Bunyips/BunyipsLib/blob/b0662cde8c7231a8e5ea4edcf72b1cf4042afe6a/src/main/java/org/murraybridgebunyips/bunyipslib/Inches.java)
 */
object Inches {
    @JvmStatic
    fun toCM(inches: Double): Double {
        return inches * 2.54
    }

    @JvmStatic
    fun toMM(inches: Double): Double {
        return inches * 25.4
    }

    @JvmStatic
    fun toM(inches: Double): Double {
        return inches * 0.0254
    }

    @JvmStatic
    fun toFieldTiles(inches: Double): Double {
        return inches / 23
    }

    @JvmStatic
    fun toHammerUnits(inches: Double): Double {
        return inches / 0.75
    }

    @JvmStatic
    fun toFootballFields(inches: Double): Double {
        return inches * 3600.0
    }

    @JvmStatic
    fun toLightYears(inches: Double): Double {
        return inches / 3.725e+17
    }

    @JvmStatic
    fun fromCM(cm: Double): Double {
        return cm / 2.54
    }

    @JvmStatic
    fun fromMM(mm: Double): Double {
        return mm / 25.4
    }

    @JvmStatic
    fun fromM(m: Double): Double {
        return m / 0.0254
    }

    @JvmStatic
    fun fromFieldTiles(tiles: Double): Double {
        return tiles * 23
    }

    @JvmStatic
    fun fromHammerUnits(units: Double): Double {
        return units * 0.75
    }

    @JvmStatic
    fun fromFootballFields(ff: Double): Double {
        return ff / 3600.0
    }

    @JvmStatic
    fun fromLightYears(ly: Double): Double {
        return ly * 3.725e+17
    }
}