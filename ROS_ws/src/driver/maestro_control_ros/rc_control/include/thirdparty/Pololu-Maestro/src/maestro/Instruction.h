#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace Maestro {
enum class Opcode;

class Instruction {
   public:
    Instruction(Opcode op, const std::string& filename, int lineNumber, int columnNumber);
    void addLiteralArgument(int value, bool isMiniMaestro);
    void setOpcode(Opcode value);
    Opcode opcode() const { return m_opcode; }
    std::vector<uint8_t> toByteList() const;
    void error(std::string msg);
    int lineNumer() const { return m_lineNumber; }
    bool isLabel() const { return m_isLabel; }
    bool isJumpToLabel() const { return m_isJumpToLabel; }
    const std::string& labelName() const { return m_labelName; }
    bool isSubroutine() const { return m_isSubroutine; }
    bool isCall() const { return m_isCall; }
    static Instruction newSubroutine(std::string name, std::string filename, int column_number, int line_number);
    static Instruction newCall(std::string name, std::string filename, int column_number, int line_number);
    static Instruction newLabel(std::string name, std::string filename, int column_number, int line_number);
    static Instruction newJumpToLabel(std::string name, std::string filename, int column_number, int line_number);
    static Instruction newConditionalJumpToLabel(std::string name, std::string filename, int column_number, int line_number);
    void completeLiterals();

   private:
    Opcode m_opcode;
    std::string m_filename;
    int m_lineNumber;
    int m_columnNumber;
    std::string m_labelName;
    bool m_isSubroutine = false;
    bool m_isCall = false;
    bool m_isLabel = false;
    bool m_isJumpToLabel = false;
    std::vector<uint16_t> m_literalArguments;
};
}  // namespace Maestro
