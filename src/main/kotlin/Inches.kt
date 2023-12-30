/**
 * Utility to convert between inches and metric units.
 * [src](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/blob/stable/TeamCode/BunyipsLib/src/main/java/org/murraybridgebunyips/bunyipslib/Inches.java)
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
}
