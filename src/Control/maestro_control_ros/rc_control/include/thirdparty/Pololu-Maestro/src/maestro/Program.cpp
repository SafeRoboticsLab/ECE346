#include "Program.h"

#include <array>
#include <iomanip>
#include <regex>
#include <sstream>

#include "Instruction.h"
#include "Opcode.h"

namespace Maestro {
const std::map<std::string, Opcode> dictionary = {{"QUIT", Opcode::QUIT},
                                                  {"LITERAL", Opcode::LITERAL},
                                                  {"LITERAL8", Opcode::LITERAL8},
                                                  {"LITERAL_N", Opcode::LITERAL_N},
                                                  {"LITERAL8_N", Opcode::LITERAL8_N},
                                                  {"RETURN", Opcode::RETURN},
                                                  {"JUMP", Opcode::JUMP},
                                                  {"JUMP_Z", Opcode::JUMP_Z},
                                                  {"DELAY", Opcode::DELAY},
                                                  {"GET_MS", Opcode::GET_MS},
                                                  {"DEPTH", Opcode::DEPTH},
                                                  {"DROP", Opcode::DROP},
                                                  {"DUP", Opcode::DUP},
                                                  {"OVER", Opcode::OVER},
                                                  {"PICK", Opcode::PICK},
                                                  {"SWAP", Opcode::SWAP},
                                                  {"ROT", Opcode::ROT},
                                                  {"ROLL", Opcode::ROLL},
                                                  {"BITWISE_NOT", Opcode::BITWISE_NOT},
                                                  {"BITWISE_AND", Opcode::BITWISE_AND},
                                                  {"BITWISE_OR", Opcode::BITWISE_OR},
                                                  {"BITWISE_XOR", Opcode::BITWISE_XOR},
                                                  {"SHIFT_RIGHT", Opcode::SHIFT_RIGHT},
                                                  {"SHIFT_LEFT", Opcode::SHIFT_LEFT},
                                                  {"LOGICAL_NOT", Opcode::LOGICAL_NOT},
                                                  {"LOGICAL_AND", Opcode::LOGICAL_AND},
                                                  {"LOGICAL_OR", Opcode::LOGICAL_OR},
                                                  {"NEGATE", Opcode::NEGATE},
                                                  {"PLUS", Opcode::PLUS},
                                                  {"MINUS", Opcode::MINUS},
                                                  {"TIMES", Opcode::TIMES},
                                                  {"DIVIDE", Opcode::DIVIDE},
                                                  {"MOD", Opcode::MOD},
                                                  {"POSITIVE", Opcode::POSITIVE},
                                                  {"NEGATIVE", Opcode::NEGATIVE},
                                                  {"NONZERO", Opcode::NONZERO},
                                                  {"EQUALS", Opcode::EQUALS},
                                                  {"NOT_EQUALS", Opcode::NOT_EQUALS},
                                                  {"MIN", Opcode::MIN},
                                                  {"MAX", Opcode::MAX},
                                                  {"LESS_THAN", Opcode::LESS_THAN},
                                                  {"GREATER_THAN", Opcode::GREATER_THAN},
                                                  {"SERVO", Opcode::SERVO},
                                                  {"SERVO_8BIT", Opcode::SERVO_8BIT},
                                                  {"SPEED", Opcode::SPEED},
                                                  {"ACCELERATION", Opcode::ACCELERATION},
                                                  {"GET_POSITION", Opcode::GET_POSITION},
                                                  {"GET_MOVING_STATE", Opcode::GET_MOVING_STATE},
                                                  {"LED_ON", Opcode::LED_ON},
                                                  {"LED_OFF", Opcode::LED_OFF},
                                                  {"PWM", Opcode::PWM},
                                                  {"PEEK", Opcode::PEEK},
                                                  {"POKE", Opcode::POKE},
                                                  {"SERIAL_SEND_BYTE", Opcode::SERIAL_SEND_BYTE},
                                                  {"CALL", Opcode::CALL}};

Program::Program(const std::string& program, bool isMiniMaestro) {
    const std::regex eol_re("\\n|\\r\\n");
    for (std::sregex_token_iterator line_iter(program.begin(), program.end(), eol_re, -1); line_iter != std::sregex_token_iterator(); ++line_iter) {
        std::string text_line = *line_iter;
        m_sourceLines.push_back(text_line);
    }

    Mode mode = Mode::NORMAL;

    int line_number = 0;
    int column_number = 0;
    const std::regex ws_re("\\s|\\t");
    for (auto text_line : m_sourceLines) {
        column_number = 1;

        // remove comments
        text_line = std::regex_replace(text_line, std::regex("#.*"), "");

        for (std::sregex_token_iterator token_iter(text_line.begin(), text_line.end(), ws_re, -1); token_iter != std::sregex_token_iterator();
             ++token_iter) {
            std::string token = *token_iter;
            if (token.empty()) {
                column_number++;
                continue;
            }
            // To upper case
            std::transform(token.begin(), token.end(), token.begin(), ::toupper);

            if (mode == Mode::NORMAL) {
                parseString(token, "script", line_number, column_number, isMiniMaestro, mode);
            } else if (mode == Mode::GOTO) {
                parseGoto(token, "script", line_number, column_number, mode);
            } else if (mode == Mode::SUBROUTINE) {
                parseSubroutine(token, "script", line_number, column_number, mode);
            }
            column_number += token.size() + 1;
        }
        line_number++;
    }
    if (!m_openBlocks.empty()) {
        const std::string currentBlockStartLabel = getCurrentBlockStartLabel();
        Instruction& bytecodeInstruction = findLabel(currentBlockStartLabel);
        bytecodeInstruction.error("BEGIN block was never closed.");
    }
    completeLiterals();
    completeCalls(isMiniMaestro);
    completeJumps();
}

std::string Program::toString() const {
    if (m_instructionList.empty()) return {};

    std::ostringstream streamWriter;
    int num = 0;
    int num2 = 0;
    Instruction bytecodeInstruction(m_instructionList[num]);

    for (size_t line_number = 0; line_number < m_sourceLines.size(); line_number++) {
        int column_number = 0;
        streamWriter << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << num2 << ": ";
        while (bytecodeInstruction.lineNumer() == line_number) {
            for (uint8_t item : bytecodeInstruction.toByteList()) {
                streamWriter << std::setw(2) << int(item);
                num2++;
                column_number += 2;
            }
            num++;
            if (num >= m_instructionList.size()) break;
            bytecodeInstruction = m_instructionList[num];
        }
        for (int j = 0; j < 20 - column_number; j++) {
            streamWriter << " ";
        }
        streamWriter << " -- " << m_sourceLines[line_number] << std::endl;
    }
    streamWriter << std::endl;
    streamWriter << "Subroutines:" << std::endl;
    streamWriter << "Hex Decimal Address Name" << std::endl;
    std::array<std::string, 128> array;
    for (const auto& subroutineAddress : m_subroutineAddresses) {
        std::string key = subroutineAddress.first;
        if (m_subroutineCommands.at(key) != Opcode::CALL) {
            uint8_t b = (uint8_t)(int(m_subroutineCommands.at(key)) - 128);
            uint16_t num4 = m_subroutineAddresses.at(key);

            std::ostringstream str;
            str << std::uppercase << std::setfill('0') << std::hex << std::setw(2) << int(b) << "  " << std::dec << std::setw(3) << int(b) << "     "
                << std::hex << std::setw(4) << num4 << "    " << key << std::endl;
            array[b] = str.str();
        }
    }
    for (const auto& str : array) {
        streamWriter << str;
    }
    for (const auto& subroutineAddress : m_subroutineAddresses) {
        const std::string key = subroutineAddress.first;
        if (m_subroutineCommands.at(key) == Opcode::CALL) {
            streamWriter << "--  ---     " << std::hex << std::setw(4) << m_subroutineAddresses.at(key) << "    " << key;
        }
    }
    return streamWriter.str();
}

void Program::addLiteral(int literal, const std::string& filename, int lineNumber, int columnNumber, bool isMiniMaestro) {
    if (m_instructionList.empty() || m_instructionList.back().opcode() != Opcode::LITERAL) {
        m_instructionList.push_back(Instruction(Opcode::LITERAL, filename, lineNumber, columnNumber));
    }
    m_instructionList.back().addLiteralArgument(literal, isMiniMaestro);
}

std::vector<uint8_t> Program::getByteList() const {
    std::vector<uint8_t> list;
    for (const Instruction& instruction : m_instructionList) {
        list.insert(list.end(), instruction.toByteList().begin(), instruction.toByteList().end());
    }
    return list;
}

void Program::openBlock(BlockType blocktype, const std::string& filename, int line_number, int column_number) {
    m_instructionList.push_back(Instruction::newLabel("block_start_" + std::to_string(m_maxBlock), filename, line_number, column_number));
    m_openBlocks.push(m_maxBlock);
    m_openBlockTypes.push(blocktype);
    m_maxBlock++;
}

Program::BlockType Program::getCurrentBlockType() const { return m_openBlockTypes.top(); }

std::string Program::getCurrentBlockStartLabel() const { return "block_start_" + std::to_string(m_openBlocks.top()); }

std::string Program::getCurrentBlockEndLabel() const { return "block_end_" + std::to_string(m_openBlocks.top()); }

std::string Program::getNextBlockEndLabel() const { return "block_end_" + std::to_string(m_maxBlock); }

Instruction& Program::findLabel(const std::string& name) {
    for (auto& instruction : m_instructionList) {
        if (instruction.isLabel() && instruction.labelName() == name) {
            return instruction;
        }
    }
    throw "Label not found.";
}

void Program::closeBlock(const std::string& filename, int line_number, int column_number) {
    m_instructionList.push_back(Instruction::newLabel("block_end_" + std::to_string(m_openBlocks.top()), filename, line_number, column_number));
    m_openBlocks.pop();
    m_openBlockTypes.pop();
}

void Program::completeJumps() {
    std::map<std::string, int> dictionary;
    int num = 0;
    for (Instruction& instruction : m_instructionList) {
        if (instruction.isLabel()) {
            if (dictionary.find(instruction.labelName()) != dictionary.end()) {
                instruction.error("The label " + instruction.labelName() + " has already been used.");
            }
            dictionary[instruction.labelName()] = num;
        }
        num += instruction.toByteList().size();
    }
    for (Instruction& instruction : m_instructionList) {
        try {
            if (instruction.isJumpToLabel()) {
                instruction.addLiteralArgument(dictionary.at(instruction.labelName()), false);
            }
        } catch (std::exception& e) {
            instruction.error("The label " + instruction.labelName() + " was not found.");
        }
    }
}

void Program::completeCalls(bool isMiniMaestro) {
    uint32_t num_subroutines = 128;
    for (Instruction& instruction : m_instructionList) {
        if (instruction.isSubroutine()) {
            if (m_subroutineCommands.find(instruction.labelName()) != m_subroutineCommands.end()) {
                instruction.error("The subroutine " + instruction.labelName() + " has already been defined.");
            }
            m_subroutineCommands[instruction.labelName()] = (num_subroutines >= 256) ? Opcode::CALL : Opcode(num_subroutines);
            num_subroutines++;
            if (num_subroutines > 255 && !isMiniMaestro) {
                instruction.error("Too many subroutines.  The limit for the Micro Maestro is 128.");
            }
        }
    }
    for (Instruction& instruction : m_instructionList) {
        try {
            if (instruction.isCall()) {
                instruction.setOpcode(m_subroutineCommands[instruction.labelName()]);
            }
        } catch (std::exception& e) {
            instruction.error("Did not understand '" + instruction.labelName() + "'");
        }
    }
    uint16_t address = 0;
    for (const Instruction& instruction : m_instructionList) {
        if (instruction.isSubroutine()) {
            m_subroutineAddresses[instruction.labelName()] = address;
        }
        address += instruction.toByteList().size();
    }
    for (Instruction& instruction : m_instructionList) {
        if (instruction.opcode() == Opcode::CALL) {
            instruction.addLiteralArgument(m_subroutineAddresses[instruction.labelName()], isMiniMaestro);
        }
    }
}

void Program::completeLiterals() {
    for (Instruction& instruction : m_instructionList) {
        instruction.completeLiterals();
    }
}

uint16_t oneByteCRC(uint8_t v) {
    const uint16_t CRC16_POLY = 40961;
    uint16_t num = v;
    for (int i = 0; i < 8; i++) {
        num = (((num & 1) != 1) ? ((uint16_t)(num >> 1)) : ((uint16_t)((uint32_t)(num >> 1) ^ CRC16_POLY)));
    }
    return num;
}

uint16_t CRC(const std::vector<uint8_t>& message) {
    uint16_t num = 0;
    for (uint16_t num2 = 0; num2 < message.size(); num2 = (uint16_t)(num2 + 1)) {
        num = (uint16_t)((num >> 8) ^ oneByteCRC((uint8_t)((uint8_t)num ^ message[num2])));
    }
    return num;
}

uint16_t Program::getCRC() const {
    std::vector<uint8_t> list;
    std::array<uint16_t, 128> array;
    for (const auto& keyValue : m_subroutineCommands) {
        if (m_subroutineCommands.at(keyValue.first) != Opcode::CALL) {
            array[uint8_t(m_subroutineCommands.at(keyValue.first)) - 128] = m_subroutineAddresses.at(keyValue.first);
        }
    }
    for (uint16_t num : array) {
        list.push_back((uint8_t)(num & 0xFFu));
        list.push_back((uint8_t)(num >> 8));
    }
    list.insert(list.end(), getByteList().begin(), getByteList().end());
    return CRC(list);
}

void Program::parseGoto(std::string s, std::string filename, int line_number, int column_number, Mode& mode) {
    m_instructionList.push_back(Instruction::newJumpToLabel("USER_" + s, filename, line_number, column_number));
    mode = Mode::NORMAL;
}

void Program::parseSubroutine(std::string s, std::string filename, int line_number, int column_number, Mode& mode) {
    if (looksLikeLiteral(s)) {
        throw "The name " + s + " is not valid as a subroutine name (it looks like a number).";
    }
    if (dictionary.find(s) != dictionary.end()) {
        throw "The name " + s + " is not valid as a subroutine name (it is a built-in command).";
    }
    const std::vector<std::string> keywords = {"GOTO", "SUB", "BEGIN", "WHILE", "REPEAT", "IF", "ENDIF", "ELSE"};
    for (std::string keyword : keywords) {
        if (keyword == s) {
            throw "The name " + s + " is not valid as a subroutine name (it is a keyword).";
        }
    }
    m_instructionList.push_back(Instruction::newSubroutine(s, filename, line_number, column_number));
    mode = Mode::NORMAL;
}

bool Program::looksLikeLiteral(const std::string& s) {
    return std::regex_match(s, std::regex("^-?[0-9.]+$")) || std::regex_match(s, std::regex("^0[xX][0-9a-fA-F.]+$"));
}

void Program::parseString(std::string s, std::string filename, int line_number, int column_number, bool isMiniMaestro, Mode& mode) {
    try {
        if (looksLikeLiteral(s)) {
            int num;
            if (s.rfind("0X", 0) == 0) {
                num = std::stoi(s, nullptr, 16);
                if (num > 65535 || num < 0) {
                    throw "Value " + s + " is not in the allowed range of " + std::to_string(std::numeric_limits<uint16_t>::min()) + " to " +
                        std::to_string(std::numeric_limits<uint16_t>::max()) + ".";
                }
            } else {
                num = std::stoi(s);
                if (num > 32767 || num < -32768) {
                    throw "Value " + s + " is not in the allowed range of " + std::to_string(std::numeric_limits<int16_t>::min()) + " to " +
                        std::to_string(std::numeric_limits<int16_t>::max()) + ".";
                }
            }
            int literal = (int16_t)(long)(num % 65535);
            addLiteral(literal, filename, line_number, column_number, isMiniMaestro);
            return;
        }
    } catch (std::exception& ex) {
        throw "Error parsing " + s + ": " + ex.what();
    }
    if (s == "GOTO") {
        mode = Mode::GOTO;
        return;
    }
    if (s == "SUB") {
        mode = Mode::SUBROUTINE;
        return;
    }
    std::smatch base_match;
    const std::regex base_regex("(.*):$");
    if (std::regex_match(s, base_match, base_regex)) {
        m_instructionList.push_back(Instruction::newLabel("USER_" + base_match[1].str(), filename, line_number, column_number));
        return;
    }
    if (s == "BEGIN") {
        openBlock(BlockType::BEGIN, filename, line_number, column_number);
        return;
    }
    if (s == "WHILE") {
        if (getCurrentBlockType() != BlockType::BEGIN) {
            throw "WHILE must be inside a BEGIN...REPEAT block";
        }
        m_instructionList.push_back(Instruction::newConditionalJumpToLabel(getCurrentBlockEndLabel(), filename, line_number, column_number));
        return;
    }
    if (s == "REPEAT") {
        try {
            if (getCurrentBlockType() != BlockType::BEGIN) {
                throw "REPEAT must end a BEGIN...REPEAT block";
            }
            m_instructionList.push_back(Instruction::newJumpToLabel(getCurrentBlockStartLabel(), filename, line_number, column_number));
            closeBlock(filename, line_number, column_number);
        } catch (std::exception& ex) {
            throw filename + ":" + std::to_string(line_number) + ":" + std::to_string(column_number) + ": Found REPEAT without a corresponding BEGIN";
        }
        return;
    }
    if (s == "IF") {
        openBlock(BlockType::IF, filename, line_number, column_number);
        m_instructionList.push_back(Instruction::newConditionalJumpToLabel(getCurrentBlockEndLabel(), filename, line_number, column_number));
        return;
    }
    if (s == "ENDIF") {
        try {
            if (getCurrentBlockType() != BlockType::IF && getCurrentBlockType() != BlockType::ELSE) {
                throw "ENDIF must end an IF...ENDIF or an IF...ELSE...ENDIF block.";
            }
            closeBlock(filename, line_number, column_number);
        } catch (std::exception& ex) {
            throw filename + ":" + std::to_string(line_number) + ":" + std::to_string(column_number) + ": Found ENDIF without a corresponding IF";
        }
        return;
    }
    if (s == "ELSE") {
        try {
            if (getCurrentBlockType() != BlockType::IF) {
                throw "ELSE must be part of an IF...ELSE...ENDIF block.";
            }
            m_instructionList.push_back(Instruction::newJumpToLabel(getNextBlockEndLabel(), filename, line_number, column_number));
            closeBlock(filename, line_number, column_number);
            openBlock(BlockType::ELSE, filename, line_number, column_number);
        } catch (std::exception& ex) {
            throw filename + ":" + std::to_string(line_number) + ":" + std::to_string(column_number) + ": Found ELSE without a corresponding IF";
        }
        return;
    }
    if (dictionary.find(s) != dictionary.end()) {
        const Opcode opcode = dictionary.at(s);
        switch (opcode) {
            case Opcode::LITERAL:
            case Opcode::LITERAL8:
            case Opcode::LITERAL_N:
            case Opcode::LITERAL8_N:
                throw filename + ":" + std::to_string(line_number) + ":" + std::to_string(column_number) +
                    ": Literal commands may not be used directly in a program.  Integers should be entered directly.";
            case Opcode::JUMP:
            case Opcode::JUMP_Z:
                throw filename + ":" + std::to_string(line_number) + ":" + std::to_string(column_number) +
                    ": Jumps may not be used directly in a program.";
        }
        if (!isMiniMaestro && (uint8_t)opcode >= (uint8_t)Opcode::PWM) {
            throw filename + ":" + std::to_string(line_number) + ":" + std::to_string(column_number) + ": " + s +
                " is only available on the Mini Maestro 12, 18, and 24.";
        }
        m_instructionList.push_back(Instruction(opcode, filename, line_number, column_number));
    } else {
        m_instructionList.push_back(Instruction::newCall(s, filename, line_number, column_number));
    }
}
}  // namespace Maestro
