cmake_minimum_required(VERSION 3.26)
project(HiggsContractTests LANGUAGES C CXX)
enable_testing()

# Compile the checkout's production core directly. No downloaded fixtures,
# generated SDK headers, SKSE, Havok, Skyrim installation or game launch.
get_filename_component(HIGGS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(HIGGS_CORE_SOURCES
    main/native_regions.cpp
    controller_tracking.cpp motion_runtime.cpp
    synergy_core.cpp synergy_runtime.cpp
    hand_broker.cpp hand_runtime.cpp hand_sessions.cpp hand_use_proposals.cpp
    hand_equipment.cpp hand_transactions.cpp transaction_runtime.cpp inventory_service.cpp
    hand_physical_policy.cpp physical_policy_runtime.cpp
    own_contact.cpp own_contact_runtime.cpp
    haptic_schedule.cpp hand_support.cpp
    hand_observations.cpp observation_runtime.cpp
    hand_input_extended.cpp hand_input_context.cpp input_runtime.cpp
    hand_parts.cpp hand_parts_runtime.cpp
)
list(TRANSFORM HIGGS_CORE_SOURCES PREPEND "${HIGGS_ROOT}/src/")
add_library(higgs_contract_core STATIC ${HIGGS_CORE_SOURCES})
target_include_directories(higgs_contract_core PUBLIC "${HIGGS_ROOT}/include")
target_compile_features(higgs_contract_core PUBLIC cxx_std_20)
set_target_properties(higgs_contract_core PROPERTIES CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
target_link_libraries(higgs_contract_core PUBLIC Threads::Threads)
if(MSVC)
    target_compile_options(higgs_contract_core PUBLIC /EHsc)
endif()

function(higgs_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE higgs_contract_core)
    target_compile_features(${name} PRIVATE c_std_11)
    set_target_properties(${name} PROPERTIES C_EXTENSIONS OFF CXX_EXTENSIONS OFF)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 60 LABELS "core;contract")
endfunction()

higgs_test(hand_broker hand_broker_tests.cpp hand_c_consumer.c)
higgs_test(hand_authority hand_authority_tests.cpp)
higgs_test(shared_rounds shared_rounds_tests.cpp)
higgs_test(native_regions native_regions_tests.cpp)
higgs_test(transactions transaction_tests.cpp)
higgs_test(equipment_claims equipment_claims_tests.cpp)
higgs_test(support support_tests.cpp)
higgs_test(physical_policy physical_policy_tests.cpp physical_policy_c_consumer.c)
higgs_test(observations observation_tests.cpp observation_c_consumer.c)
higgs_test(input input_tests.cpp input_c_consumer.c)
higgs_test(hand_parts hand_parts_tests.cpp hand_parts_c_consumer.c)
higgs_test(tracking controller_tracking_tests.cpp)
higgs_test(motion motion_abi_tests.cpp motion_c_consumer.c)
higgs_test(restrictions synergy_tests.cpp synergy_c_consumer.c)
higgs_test(inventory_service inventory_service_tests.cpp inventory_c_consumer.c)
higgs_test(own_contacts own_contact_tests.cpp own_contact_c_consumer.c)
