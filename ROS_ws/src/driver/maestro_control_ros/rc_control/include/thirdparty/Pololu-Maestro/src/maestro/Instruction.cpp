#include "Instruction.h"

#include "Opcode.h"

namespace Maestro {
Instruction::Instruction(Opcode op, const std::string& filename, int lineNumber, int columnNumber)
    : m_opcode(op), m_filename(filename), m_lineNumber(lineNumber), m_columnNumber(columnNumber) {}

void Instruction::addLiteralArgument(int value, bool isMiniMaestro) {
    m_literalArguments.push_back(value);
    if (!isMiniMaestro && m_literalArguments.size() > 32) {
        throw "Too many literals (> 32) in a row: this will overflow the stack.";
    }
    if (m_literalArguments.size() > 126) {
        throw "Too many literals (> 126) in a row: this will overflow the stack.";
    }
}

void Instruction::setOpcode(Opcode value) {
    if (m_opcode != Opcode::QUIT) {
        throw "The opcode has already been set.";
    }
    m_opcode = value;
}

std::vector<uint8_t> Instruction::toByteList() const {
    std::vector<uint8_t> list;
    if (m_isLabel || m_isSubroutine) {
        return list;
    }
    list.push_back((uint8_t)m_opcode);
    if (m_opcode == Opcode::LITERAL || m_opcode == Opcode::JUMP || m_opcode == Opcode::JUMP_Z || m_opcode == Opcode::CALL) {
        if (m_literalArguments.empty()) {
            list.push_back(0);
            list.push_back(0);
        } else {
            list.push_back(uint8_t(m_literalArguments[0] % 256));
            list.push_back(uint8_t(m_literalArguments[0] / 256));
        }
    } else if (m_opcode == Opcode::LITERAL8) {
        list.push_back(uint8_t(m_literalArguments[0]));
    } else {
        if (m_opcode == Opcode::LITERAL_N) {
            list.push_back(uint8_t(m_literalArguments.size() * 2));
            for (uint16_t literalArgument : m_literalArguments) {
                list.push_back(uint8_t(literalArgument % 256));
                list.push_back(uint8_t(literalArgument / 256));
            }
        }
        if (m_opcode == Opcode::LITERAL8_N) {
            list.push_back(uint8_t(m_literalArguments.size()));
            for (uint16_t literalArgument : m_literalArguments) {
                list.push_back(uint8_t(literalArgument));
            }
        }
    }
    return list;
}

void Instruction::error(std::string msg) {
    throw m_filename + ":" + std::to_string(m_lineNumber) + ":" + std::to_string(m_columnNumber) + ": " + msg;
}

Instruction Instruction::newSubroutine(std::string name, std::string filename, int column_number, int line_number) {
    Instruction bytecodeInstruction(Opcode::QUIT, filename, column_number, line_number);
    bytecodeInstruction.m_isSubroutine = true;
    bytecodeInstruction.m_labelName = name;
    return bytecodeInstruction;
}

Instruction Instruction::newCall(std::string name, std::string filename, int column_number, int line_number) {
    Instruction bytecodeInstruction(Opcode::QUIT, filename, column_number, line_number);
    bytecodeInstruction.m_isCall = true;
    bytecodeInstruction.m_labelName = name;
    return bytecodeInstruction;
}

Instruction Instruction::newLabel(std::string name, std::string filename, int column_number, int line_number) {
    Instruction bytecodeInstruction(Opcode::QUIT, filename, column_number, line_number);
    bytecodeInstruction.m_isLabel = true;
    bytecodeInstruction.m_labelName = name;
    return bytecodeInstruction;
}

Instruction Instruction::newJumpToLabel(std::string name, std::string filename, int column_number, int line_number) {
    Instruction bytecodeInstruction(Opcode::JUMP, filename, column_number, line_number);
    bytecodeInstruction.m_isJumpToLabel = true;
    bytecodeInstruction.m_labelName = name;
    return bytecodeInstruction;
}

Instruction Instruction::newConditionalJumpToLabel(std::string name, std::string filename, int column_number, int line_number) {
    Instruction bytecodeInstruction(Opcode::JUMP_Z, filename, column_number, line_number);
    bytecodeInstruction.m_isJumpToLabel = true;
    bytecodeInstruction.m_labelName = name;
    return bytecodeInstruction;
}

void Instruction::completeLiterals() {
    if (m_opcode != Opcode::LITERAL) {
        return;
    }
    bool flag = false;
    for (uint16_t literalArgument : m_literalArguments) {
        if (literalArgument > 255 || literalArgument < 0) {
            flag = true;
            break;
        }
    }
    if (flag && m_literalArguments.size() > 1) {
        m_opcode = Opcode::LITERAL_N;
    } else if (flag && m_literalArguments.size() == 1) {
        m_opcode = Opcode::LITERAL;
    } else if (m_literalArguments.size() > 1) {
        m_opcode = Opcode::LITERAL8_N;
    } else {
        m_opcode = Opcode::LITERAL8;
    }
}
}  // namespace Maestro
