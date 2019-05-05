import java.io.Closeable
import java.io.FileWriter
import java.lang.RuntimeException
import java.nio.file.Files
import java.nio.file.Paths

class CSVWriter(val fileName: String,
                val fieldNames: List<String>) : Closeable {

    private val fileWriter = if (Files.exists(Paths.get(fileName)))
        throw RuntimeException("File $fileName exists")
    else
        FileWriter(fileName)

    init {
        fileWriter.appendln(fieldNames.joinToString(","))
    }

    /**
     * Adds a new entry to this CSV file.
     */
    fun writeRecord(values: Map<String, Any>) {
        if (values.keys != HashSet(fieldNames))
            throw RuntimeException("Given record has keys ${values.keys}, expected keys $fieldNames")

        fileWriter.appendln(fieldNames
            .map { values[it] }
            .joinToString ( "," ))
    }

    override fun close() = fileWriter.close()
}